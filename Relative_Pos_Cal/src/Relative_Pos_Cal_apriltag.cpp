#include "Relative_Pos_Cal_apriltag.h"
#include <utility>

using namespace std;
using namespace std::chrono;
// 目标：相对于图像时间戳固定 40 ms 延时（你可以改成 0.05）
constexpr double PROCESSING_LATENCY = 0.04; // 40 ms

// 相机坐标系到无人机机体系（IMU）的小偏移
const Eigen::Vector3d POS_OFFSET(0.0, -0.0712, 0.01577);

// 简单的 90° 旋转函数（与你原来的保持一致）
template <typename T>
void rotate90(const vpImage<T> &src, vpImage<T> &dst, bool clockwise)
{
  const unsigned int H = src.getHeight();
  const unsigned int W = src.getWidth();

  // 旋转后高宽互换
  dst.resize(W, H);

  if (clockwise)
  {
    // 顺时针: (i, j) -> (j, H-1-i)
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

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
  : nh_(nh),
    stop_thread(false),
    lost_target(true),
    desired_yaw(0.0),
    clockwise(true)
{
  // ----------------- 1. RealSense 初始化 -----------------
  config.disable_stream(RS2_STREAM_DEPTH);
  config.disable_stream(RS2_STREAM_INFRARED);
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);

  try
  {
    rs2::pipeline_profile profile;

    if (!g.open(config))
    {
      ROS_ERROR_STREAM("ObjectDetector: Failed to open RealSense grabber!");
      throw std::runtime_error("Failed to open RealSense grabber!");
    }

    profile = g.getPipelineProfile();

    // ISP / 传感器（D405 的 ISP 在 depth_sensor 模块里）
    auto dev = profile.get_device();
    auto sensor = dev.first<rs2::depth_sensor>();

    // 关闭自动曝光
    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
    {
      sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    }

    // 手动曝光时间（微秒）
    if (sensor.supports(RS2_OPTION_EXPOSURE))
    {
      sensor.set_option(RS2_OPTION_EXPOSURE, 3000); // 3 ms
    }

    // 增益
    if (sensor.supports(RS2_OPTION_GAIN))
    {
      sensor.set_option(RS2_OPTION_GAIN, 64);
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("ObjectDetector: Failed to initialize RealSense grabber: " << e.what());
    throw;
  }

  // ----------------- 2. 相机内参 & 旋转后相机模型 -----------------
  cam = g.getCameraParameters(RS2_STREAM_COLOR,
                              vpCameraParameters::perspectiveProjWithoutDistortion);

  double fx = cam.get_px();
  double fy = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  unsigned int W = 640; // 旋转前宽度
  unsigned int H = 480; // 旋转前高度

  if (clockwise)
  {
    // 你原来用的 Kannala-Brandt 标定参数，直接沿用
    cam_rot.initProjWithKannalaBrandtDistortion(
      392.5919623095264, 394.6504822990919,
      238.78379511684258, 322.6289796142194,
      std::vector<double>{
        0.29186846588871074,
        0.1618033745553163,
        0.065934344194105,
        0.0270748559249096});
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

  // ----------------- 3. AprilTag 检测器配置 -----------------
  tag_detector.setAprilTagQuadDecimate(2);
  tag_detector.setAprilTagPoseEstimationMethod(
      vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  // ----------------- 4. 位姿相关变量初始化 -----------------
  Position_before = Eigen::Vector3d::Zero();
  Position_after  = Eigen::Vector3d::Zero();

  // IMU -> camera 的旋转矩阵（你原来的数值）
  R_i2c << 0.0141, -0.9999, 0.0078,
          -0.9997, -0.0142, -0.0181,
           0.0183, -0.0076, -0.9998;

  // 做一下正规化，避免数值上不是严格旋转矩阵
  {
    Eigen::Quaterniond q_i2c(R_i2c);
    R_i2c = q_i2c.normalized().toRotationMatrix();
  }

  R_w2c = R_i2c;
  R_c2a = Eigen::Matrix3d::Identity();
  R_w2a = Eigen::Matrix3d::Identity();

  // ----------------- 5. ROS 通信 -----------------
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "/point_with_fixed_delay", 1, true);

  odom_sub_ = nh_.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 1,
      &ObjectDetector::odomCallback, this,
      ros::TransportHints().tcpNoDelay());

  image_pub = nh_.advertise<sensor_msgs::Image>(
      "/camera/image", 1);

  // ----------------- 6. 后台处理线程 -----------------
  worker_thread = std::thread(&ObjectDetector::processImages, this);
}

ObjectDetector::~ObjectDetector()
{
  // 通知线程退出
  stop_thread.store(true, std::memory_order_release);

  // 先关闭 grabber，避免 acquire 一直卡住
  try
  {
    g.close();
  }
  catch (...)
  {
    // 析构里不要让异常往外跑
  }

  if (worker_thread.joinable())
  {
    worker_thread.join();
  }
}

void ObjectDetector::start()
{
  // 主线程跑 ROS 回调（IMU 等）
  ros::spin();
}

void ObjectDetector::odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);

  Eigen::Matrix3d R_new = q.toRotationMatrix() * R_i2c;

  std::lock_guard<std::mutex> lock(R_mutex);
  R_w2c = R_new;
}

void ObjectDetector::publishDetectionResult(ros::Time &ts, bool is_fault)
{
  static std_msgs::Float64MultiArray msg;
  if (msg.data.size() != 6)
    msg.data.resize(6);

  msg.data[0] = Position_after.x();
  msg.data[1] = Position_after.y();
  msg.data[2] = Position_after.z();
  msg.data[3] = ts.toSec();
  msg.data[4] = static_cast<double>(is_fault || lost_target);
  msg.data[5] = desired_yaw;

  point_pub_.publish(msg);

  if (is_fault)
  {
    ROS_WARN_STREAM("Fault detected, skipping processing.");
  }
}

void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  // 目标：从 ts 开始，延迟 PROCESSING_LATENCY 秒再发布
  ros::Duration delay =
      ros::Duration(PROCESSING_LATENCY) - (ros::Time::now() - ts);

  if (delay.toSec() > 0.0)
  {
    delay.sleep();
  }

  publishDetectionResult(ts, is_fault);
}

void ObjectDetector::processImages()
{
  while (ros::ok() && !stop_thread.load(std::memory_order_acquire))
  {
    ros::Time image_timestamp;
    bool fault_detected = false;

    try
    {
      // 1. 采集图像
      vpImage<unsigned char> I;
      g.acquire(I);
      image_timestamp = ros::Time::now();

      // 2. 旋转图像
      vpImage<unsigned char> rotated;
      rotate90(I, rotated, clockwise);
      std::swap(I, rotated);

      // 3. 发布调试图像（mono8）
      cv::Mat imageMat;
      vpImageConvert::convert(I, imageMat);

      std_msgs::Header header;
      header.stamp = image_timestamp;
      sensor_msgs::ImagePtr img_msg =
          cv_bridge::CvImage(header, "mono8", imageMat).toImageMsg();
      image_pub.publish(img_msg);

      // 4. AprilTag 检测
      static std::vector<vpHomogeneousMatrix> cMo_vec;
      cMo_vec.clear();
      cMo_vec.reserve(1);

      tag_detector.detect(I, 0.0561, cam_rot, cMo_vec);

      fault_detected = (cMo_vec.size() != 1);

      if (!fault_detected)
      {
        // 相机坐标系下的 Tag 位置
        Position_before[0] = cMo_vec[0][0][3];
        Position_before[1] = cMo_vec[0][1][3];
        Position_before[2] = cMo_vec[0][2][3];

        // 加上相机到机体的平移偏移
        Position_before += POS_OFFSET;

        // 当前世界到相机的旋转，从回调里拿
        Eigen::Matrix3d R_w2c_local;
        {
          std::lock_guard<std::mutex> lock(R_mutex);
          R_w2c_local = R_w2c;
        }

        // 世界坐标系下的位置
        Position_after = R_w2c_local * Position_before;

        // 相机到 AprilTag 的旋转
        vpRotationMatrix R_c2a_vp = cMo_vec[0].getRotationMatrix();
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            R_c2a(i, j) = R_c2a_vp[i][j];

        R_w2a = R_w2c_local * R_c2a;
        Eigen::Quaterniond q_w2a(R_w2a);

        // 从四元数里提取 yaw（绕 Z），再加 90 度偏置
        double siny_cosp = 2.0 * (q_w2a.w() * q_w2a.z() +
                                  q_w2a.x() * q_w2a.y());
        double cosy_cosp = 1.0 - 2.0 * (q_w2a.y() * q_w2a.y() +
                                        q_w2a.z() * q_w2a.z());
        desired_yaw = std::atan2(siny_cosp, cosy_cosp) + M_PI / 2.0;

        lost_target = false;

        static int print_count = 0;
        if (++print_count % 10 == 0)
        {
          std::cout << "[Aruco] Position_after: "
                    << Position_after.transpose() << std::endl;
        }
      }
      else
      {
        lost_target = true;
      }
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Processing error: " << e.what());
      image_timestamp = ros::Time::now();
      fault_detected = true;
    }

    // 5. 相对于图像时间戳的固定延时 + 发布
    baseProcess(image_timestamp, fault_detected);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");  // 节点名
  ros::NodeHandle nh;

  ObjectDetector detector(nh);
  detector.start();

  return 0;
}

