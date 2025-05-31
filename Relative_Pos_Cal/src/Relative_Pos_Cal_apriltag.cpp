#include "Relative_Pos_Cal_apriltag.h"

using namespace std;
using namespace std::chrono;
template <typename T>
void rotate90(const vpImage<T> &src, vpImage<T> &dst, bool clockwise)
{
  const unsigned int H = src.getHeight();
  const unsigned int W = src.getWidth();
  // 旋转后高宽互换
  dst.resize(W, H);

  if (clockwise)
  {
    // (i, j) -> (j, H-1-i)
    for (unsigned int i = 0; i < H; ++i)
    {
      for (unsigned int j = 0; j < W; ++j)
      {
        dst[j][H - 1 - i] = src[i][j];
      }
    }
  }
  else
  {
    // 逆时针: (i, j) -> (W-1-j, i)
    for (unsigned int i = 0; i < H; ++i)
    {
      for (unsigned int j = 0; j < W; ++j)
      {
        dst[W - 1 - j][i] = src[i][j];
      }
    }
  }
}

constexpr double PROCESSING_LATENCY = 0.04; // 40ms处理延迟
const Eigen::Vector3d POS_OFFSET{0.00625, -0.08892, 0.06430};
ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh), stop_thread(false), processing(false),
      lost_target(true), desired_yaw(0), clockwise(true)
{
  config.disable_stream(RS2_STREAM_DEPTH);
  config.disable_stream(RS2_STREAM_INFRARED);
  config.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_RGB8, 30);
  try
  {
    rs2::pipeline_profile profile;
    // 1.1 打开管线（只开 Color，640×360，RGB8，30fps）
    if (!g.open(config))
    {
      // 如果打开失败，抛出异常
      ROS_ERROR_STREAM("ObjectDetector: Failed to open RealSense grabber!");
      throw std::runtime_error("Failed to open RealSense grabber!");
    }

    // 1.2 拿到 ViSP 内部创建的 rs2::pipeline_profile
    profile = g.getPipelineProfile();

    // 1.3 通过 profile 拿 device → depth_sensor（D405 的 ISP 在 depth 传感器模块里）
    auto dev = profile.get_device();
    auto sensor = dev.first<rs2::depth_sensor>();

    // 1.4 关闭自动曝光
    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
    {
      sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    }

    // 1.5 手动设曝光时间（单位：微秒），这里设 3000 μs ≈ 3ms
    if (sensor.supports(RS2_OPTION_EXPOSURE))
    {
      sensor.set_option(RS2_OPTION_EXPOSURE, 3000);
    }

    // 1.6 提高增益以补偿亮度下降（可选）
    if (sensor.supports(RS2_OPTION_GAIN))
    {
      sensor.set_option(RS2_OPTION_GAIN, 64);
    }
    // 这样就把曝光、增益等参数“真正”地作用到了 ViSP 的管线上，而且只做了一次。
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("ObjectDetector: Failed to initialize RealSense grabber: " << e.what());
    throw; // 或自行处理，决定是否退出程序
  }
  cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  // 假设原 cam 已经由 g.getCameraParameters(...) 构造
  double fx = cam.get_px();
  double fy = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  unsigned int W = 640; // 旋转前宽度
  unsigned int H = 360; // 旋转前高度

  if (clockwise)
  {
    // 顺时针 90°
    cam_rot.initPersProjWithoutDistortion(
        /*fx=*/fy,
        /*fy=*/fx,
        /*u0=*/(double)(H - 1) - v0,
        /*v0=*/u0);
  }
  else
  {
    // 逆时针 90°
    cam_rot.initPersProjWithoutDistortion(
        /*fx=*/fy,
        /*fy=*/fx,
        /*u0=*/v0,
        /*v0=*/(double)(W - 1) - u0);
  }
  tag_detector.setAprilTagQuadDecimate(2);
  tag_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();
  R_i2c << 0.00698, -0.99997, 0.00279,
      -0.99988, -0.00694, 0.01416,
      -0.01414, -0.00289, -0.99990;
  R_w2c = R_i2c;
  R_c2a = Eigen::Matrix3d::Identity();
  R_w2a = Eigen::Matrix3d::Identity();

  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);
  odom_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
  image_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 1);
  timer = nh_.createTimer(ros::Duration(0.05), &ObjectDetector::timerCallback, this);
  worker_thread = thread(&ObjectDetector::processImages, this);
}

ObjectDetector::~ObjectDetector()
{
  stop_thread.store(true, std::memory_order_release);
  cv.notify_all();
  if (worker_thread.joinable())
  {
    worker_thread.join();
  }
  g.close();
}

void ObjectDetector::start()
{
  ros::spin();
  ros::waitForShutdown();
}

void ObjectDetector::timerCallback(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> lock(data_mutex);
  processing.store(true, std::memory_order_release);
  cv.notify_one();
}

void ObjectDetector::odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->orientation.w, msg->orientation.x,
      msg->orientation.y, msg->orientation.z);
  R_w2c = q.toRotationMatrix() * R_i2c;
}

void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  const auto delay =
      ros::Duration(PROCESSING_LATENCY) - (ros::Time::now() - ts);
  // cout << "Processing delay: " << delay.toSec() << " seconds." << endl;
  if (delay > ros::Duration(0))
  {
    this_thread::sleep_for(
        milliseconds(static_cast<long>(delay.toSec() * 1000)));
  }
  publishDetectionResult(ts, is_fault);
  processing.store(false, std::memory_order_release);
}

void ObjectDetector::publishDetectionResult(ros::Time &ts, bool is_fault)
{
  std_msgs::Float64MultiArray msg;
  msg.data = {Position_after.x(), Position_after.y(), Position_after.z(), ts.toSec(), static_cast<double>(is_fault || lost_target), desired_yaw};
  point_pub_.publish(msg);
  if (is_fault)
  {
    ROS_WARN_STREAM("Fault detected, skipping processing.");
  }
  else
  {
    ROS_INFO_STREAM("Processing completed successfully.");
  }
}

void ObjectDetector::processImages()
{
  std::unique_lock<std::mutex> lock(data_mutex);
  while (!stop_thread.load(std::memory_order_acquire))
  {
    cv.wait(lock, [&]
            { return processing.load(std::memory_order_acquire) ||
                     stop_thread.load(std::memory_order_acquire); });

    if (stop_thread.load(std::memory_order_acquire))
      break;

    processing.store(false, std::memory_order_release);
    lock.unlock(); // 解锁进入图像处理流程
    try
    {
      vpImage<unsigned char> I;
      g.acquire(I);
      ros::Time image_timestamp = ros::Time::now();
      vpImage<unsigned char> flippedImage;
      rotate90(I, flippedImage, clockwise);
      I = flippedImage;
      cv::Mat imageMat;
      vpImageConvert::convert(I, imageMat);
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", imageMat).toImageMsg();
      image_pub.publish(msg);
      std::vector<vpHomogeneousMatrix> cMo_vec;
      tag_detector.detect(I, 0.0795, cam_rot, cMo_vec);
      // ros::Time detection_time = ros::Time::now();
      // cout << "Detection time: " << detection_time.toSec() - image_timestamp.toSec() << " seconds." << endl;
      bool fault_detected = (cMo_vec.size() != 1);
      if (!fault_detected)
      {
        Position_before[0] = cMo_vec[0][0][3];
        Position_before[1] = cMo_vec[0][1][3];
        Position_before[2] = cMo_vec[0][2][3];
        Position_before += POS_OFFSET;
        Position_after = R_w2c * Position_before;

        vpRotationMatrix R_c2a_vp = cMo_vec[0].getRotationMatrix();
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            R_c2a(i, j) = R_c2a_vp[i][j];
        R_w2a = R_w2c * R_c2a;
        Eigen::Quaterniond q_w2a(R_w2a);
        desired_yaw = atan2(2 * (q_w2a.w() * q_w2a.z() + q_w2a.x() * q_w2a.y()), 1 - 2 * (q_w2a.y() * q_w2a.y() + q_w2a.z() * q_w2a.z())) + M_PI / 2;
        lost_target = false;
        cout << "Position_after: " << Position_after.transpose() << endl;
        cout << "Desired yaw: " << desired_yaw << endl;
      }
      else
      {
        // ROS_INFO_STREAM("No tag detected or multiple tags detected.");
        lost_target = true;
      }
      baseProcess(image_timestamp, fault_detected);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Processing error: " << e.what());
      baseProcess(ros::Time::now(), true);
    }
    lock.lock(); // 每次循环末尾，统一重新加锁
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ObjectDetector detector(nh);
  detector.start();
  return 0;
}
