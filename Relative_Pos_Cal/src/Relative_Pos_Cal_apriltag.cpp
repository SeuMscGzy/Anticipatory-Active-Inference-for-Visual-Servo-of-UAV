#include "Relative_Pos_Cal_apriltag.h"

using namespace std;
using namespace std::chrono;

// 常量定义
constexpr double PROCESSING_LATENCY = 0.040; // 40ms处理延迟
const Eigen::Vector3d t(0, -0.0584, 0);
const Eigen::Vector3d POS_OFFSET{0.00125, -0.03655, 0.02884};
constexpr int CAMERA_RETRY_TIMES = 3;
constexpr double CAMERA_RETRY_INTERVAL = 0.5;
constexpr double DEFAULT_FX = 426.44408;
constexpr double DEFAULT_FY = 427.70327;
constexpr double DEFAULT_CX = 344.18464;
constexpr double DEFAULT_CY = 255.63631;

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh), camera_cap("/dev/video0"),
      stop_thread(false), processing(false),
      lost_target(true), desired_yaw(0)
{
  // 参数服务器配置
  double fx, fy, cx, cy;
  nh_.param("/camera/fx", fx, DEFAULT_FX);
  nh_.param("/camera/fy", fy, DEFAULT_FY);
  nh_.param("/camera/cx", cx, DEFAULT_CX);
  nh_.param("/camera/cy", cy, DEFAULT_CY);

  cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  tag_sizes = {{0, 0.0254}, {1, 0.081}};
  // AprilTag配置优化
  tf = tag36h11_create();
  td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->nthreads = std::min(2, static_cast<int>(std::thread::hardware_concurrency()));

  // 相机初始化重试机制
  bool camera_initialized = false;
  for (int retry = 0; retry < CAMERA_RETRY_TIMES; ++retry)
  {
    if (camera_cap.init())
    {
      camera_initialized = true;
      break;
    }
    ROS_WARN("Camera init retrying...%d", retry + 1);
    ros::Duration(CAMERA_RETRY_INTERVAL).sleep();
  }
  if (!camera_initialized)
  {
    throw std::runtime_error("Camera initialization failed");
  }

  // 坐标系初始化
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();
  R_c2a = Eigen::Matrix3d::Identity();
  R_i2c << 0.00698, -0.99997, 0.00279,
      -0.99988, -0.00694, 0.01416,
      -0.01414, -0.00289, -0.99990;
  R_w2a = Eigen::Matrix3d::Identity();
  R_w2c = R_i2c;

  for (const auto &kv : tag_sizes)
  {
    int id = kv.first;
    double tag_size = kv.second;
    cached_obj_pts_[id] = {
        cv::Point3d(-tag_size / 2, tag_size / 2, 0),
        cv::Point3d(tag_size / 2, tag_size / 2, 0),
        cv::Point3d(tag_size / 2, -tag_size / 2, 0),
        cv::Point3d(-tag_size / 2, -tag_size / 2, 0)};
  }

  // ROS组件初始化
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);
  cv_image_pub = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
  timer = nh_.createTimer(ros::Duration(0.06), &ObjectDetector::timerCallback, this);
  worker_thread = thread(&ObjectDetector::processImages, this);
}

ObjectDetector::~ObjectDetector()
{
  camera_cap.release();
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  stop_thread.store(true, std::memory_order_release);
  cv.notify_all();
  if (worker_thread.joinable())
  {
    worker_thread.join();
  }
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

void ObjectDetector::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  cout << "yaw_imu:" << fromQuaternion2yaw(q) << endl;
  std::lock_guard<std::mutex> lock(data_mutex);
  R_w2c = q.toRotationMatrix() * R_i2c;
}

void ObjectDetector::processSingleTag(apriltag_detection_t *det, Eigen::Matrix3d &R_c2a, Eigen::Vector3d &position)
{
  // 从缓存中查找预计算的物体点
  auto it = cached_obj_pts_.find(det->id);
  if (it == cached_obj_pts_.end())
  {
    throw std::runtime_error("No cached object points for tag id");
  }
  const std::vector<cv::Point3d> &obj_pts = it->second;

  vector<cv::Point2d> img_pts = {
      {det->p[0][0], det->p[0][1]},
      {det->p[1][0], det->p[1][1]},
      {det->p[2][0], det->p[2][1]},
      {det->p[3][0], det->p[3][1]}};

  if (!cv::solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec_, tvec_,
                    false, cv::SOLVEPNP_IPPE_SQUARE))
  {
    throw std::runtime_error("solvePnP failed");
  }

  // 这里尝试原地转换：直接用 rvec 存储旋转矩阵
  cv::Rodrigues(rvec_, rvec_); // rvec 现在存储 3x3 的旋转矩阵
  cv::cv2eigen(rvec_, R_c2a); // 直接转换到 Eigen
  position = Eigen::Vector3d(tvec_.at<double>(0), tvec_.at<double>(1),
                             tvec_.at<double>(2));
}

void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  const auto delay =
      ros::Duration(PROCESSING_LATENCY) - (ros::Time::now() - ts);
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
}

double ObjectDetector::fromQuaternion2yaw(const Eigen::Quaterniond &q)
{
  return atan2(2 * (q.w() * q.z() + q.x() * q.y()),
               1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}

void ObjectDetector::processImages()
{
  cv::Mat distorted_image;
  while (!stop_thread.load(std::memory_order_acquire))
  {
    distorted_image.release(); // 显式释放
    Eigen::Matrix3d R_w2c_temp;
    {
      unique_lock<std::mutex> lock(data_mutex);
      // 等待 processing 标志或退出标志
      cv.wait(lock, [&]
              { return processing.load(std::memory_order_acquire) ||
                       stop_thread.load(std::memory_order_acquire); });
      if (stop_thread.load(std::memory_order_acquire))
        break;
      // 复制共享变量，然后释放锁
      R_w2c_temp = R_w2c;
    } // 锁在此处释放
    try
    {
      ros::Time image_timestamp = ros::Time::now();
      // 获取图像
      if (!camera_cap.captureImage(distorted_image) ||
          distorted_image.empty())
      {
        baseProcess(image_timestamp, true);
        continue;
      }
      // AprilTag检测
      image_u8_t apriltag_image = {.width = distorted_image.cols,
                                   .height = distorted_image.rows,
                                   .stride = static_cast<int>(distorted_image.step),
                                   .buf = distorted_image.data};
      zarray_t *raw_detections =
          apriltag_detector_detect(td, &apriltag_image);
      unique_ptr<zarray_t, decltype(&apriltag_detections_destroy)> detections(
          raw_detections, apriltag_detections_destroy);
      // 处理检测结果
      const int detection_count = zarray_size(detections.get());
      ROS_DEBUG_STREAM("Detected tags: " << detection_count);
      bool has_invalid_tag = false;
      int selected_index = -1;
      bool found_tag1 = false;
      // 第一遍遍历检测结果
      for (int i = 0; i < detection_count; ++i)
      {
        apriltag_detection_t *det;
        zarray_get(detections.get(), i, &det);
        if (tag_sizes.find(det->id) == tag_sizes.end())
        {
          has_invalid_tag = true;
          break;
        }
        if (det->id == 1)
        {
          selected_index = i;
          found_tag1 = true;
          break;
        }
        if (selected_index == -1)
        {
          selected_index = i;
        }
      }
      if (has_invalid_tag || detection_count == 0)
      {
        baseProcess(image_timestamp, true);
        continue;
      }
      // 处理选中的标签
      apriltag_detection_t *det;
      zarray_get(detections.get(), selected_index, &det);
      processSingleTag(det, R_c2a, Position_before);
      // 坐标系转换（保持原有补偿逻辑）
      R_w2a = R_w2c_temp * R_c2a;
      if (found_tag1 == true)
      {
        Position_before -= R_c2a * t;
      }
      // 应用位置偏移
      Position_before += POS_OFFSET;
      // 转换到世界坐标系
      Position_after = R_w2c_temp * Position_before;
      cout << "Position_after: " << Position_after.transpose() << endl;
      // 计算偏航角
      Eigen::Quaterniond q(R_w2a);
      double yaw = fromQuaternion2yaw(q);
      yaw += M_PI / 2;
      // 更新状态
      desired_yaw = yaw;
      //cout << "yaw: " << yaw << endl;
      lost_target = false;
      // 后续处理
      baseProcess(image_timestamp, false);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Processing error: " << e.what());
      baseProcess(ros::Time::now(), true);
    }
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
