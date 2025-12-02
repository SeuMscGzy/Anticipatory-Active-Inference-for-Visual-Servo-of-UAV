#include "Relative_Pos_Cal_apriltag.h"
#include <utility>
#include <thread> // 为 std::this_thread::sleep_for
#include <chrono>

using namespace std;
using namespace std::chrono;

// 目标：相对于图像时间戳固定 40 ms 延时
constexpr double PROCESSING_LATENCY = 0.04; // 40 ms
// 目标：整轮 while 循环周期 60 ms
const milliseconds LOOP_PERIOD(60);

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
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
  try
  {
    if (!g.open(config))
    {
      ROS_ERROR_STREAM("ObjectDetector: Failed to open RealSense grabber!");
      throw std::runtime_error("Failed to open RealSense grabber!");
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("ObjectDetector: Failed to initialize RealSense grabber: " << e.what());
    throw;
  }
  // 你原来用的 Kannala-Brandt 标定参数，直接沿用
  cam_rot.initProjWithKannalaBrandtDistortion(
      392.5919623095264, 394.6504822990919,
      238.78379511684258, 322.6289796142194,
      std::vector<double>{
          0.29186846588871074,
          0.1618033745553163,
          0.065934344194105,
          0.0270748559249096});

  // ----------------- 3. AprilTag 检测器配置 -----------------
  tag_detector.setAprilTagQuadDecimate(2);
  tag_detector.setAprilTagPoseEstimationMethod(
      vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  // ----------------- 4. 位姿相关变量初始化 -----------------
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();

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

// ============ 固定 40 ms 处理延时（相对于图像时间戳） ============
void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  // 1. 用 ROS 时间计算从图像时间戳到现在已经过去多久
  const double elapsed = (ros::Time::now() - ts).toSec();
  const double remain = PROCESSING_LATENCY - elapsed;

  // 2. 真正 sleep 用 wall time，避免 use_sim_time 或 /clock 停住导致永久阻塞
  if (remain > 0.0)
  {
    ros::WallDuration(remain).sleep();
  }

  // 如果系统已经在关闭，就别再发布了（保险一点）
  if (!ros::ok() || stop_thread.load(std::memory_order_acquire))
  {
    return;
  }

  publishDetectionResult(ts, is_fault);
}

// ============ 主处理线程：目标每 60 ms 执行一轮 while ============
void ObjectDetector::processImages()
{
  while (ros::ok() && !stop_thread.load(std::memory_order_acquire))
  {
    // 本轮循环开始时间（用于控制 60 ms 周期）
    auto loop_start = steady_clock::now();

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
        desired_yaw = atan2(siny_cosp, cosy_cosp) + M_PI / 2.0;

        lost_target = false;

        static int print_count = 0;
        if (++print_count % 10 == 0)
        {
          cout << "[Aruco] Position_after: "
               << Position_after.transpose() << std::endl;
          cout << "[Aruco] Desired yaw (rad): "
               << desired_yaw << std::endl;
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

    // ========== 控制整轮循环的周期为 60 ms ==========
    auto loop_end = steady_clock::now();
    auto elapsed = duration_cast<milliseconds>(loop_end - loop_start);

    if (elapsed < LOOP_PERIOD)
    {
      auto sleep_dur = LOOP_PERIOD - elapsed;

      // 如果在这期间已经请求退出，就不要再多 sleep 了
      if (!stop_thread.load(std::memory_order_acquire) && ros::ok())
      {
        std::this_thread::sleep_for(sleep_dur);
      }
    }
    // 如果 elapsed >= 60 ms，就不再额外 sleep，直接进入下一轮
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector"); // 节点名
  ros::NodeHandle nh;

  ObjectDetector detector(nh);
  detector.start();

  return 0;
}
