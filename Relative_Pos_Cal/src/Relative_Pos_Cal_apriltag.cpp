#include "Relative_Pos_Cal_apriltag.h"

#include <utility>
#include <iostream>
#include <cstring>
using namespace std;
// 目标：每 100 ms 处理一次
static constexpr double LOOP_HZ = 10;

// 相机坐标系到无人机机体系（IMU）的小偏移
static const Eigen::Vector3d POS_OFFSET(0.0, -0.0712, 0.01577);

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh),
      lost_target(true),
      desired_yaw(0.0),
      clockwise(true)
{
  // ----------------- 1. RealSense 原生 pipeline 初始化 -----------------
  cfg_.disable_stream(RS2_STREAM_DEPTH);
  cfg_.disable_stream(RS2_STREAM_INFRARED);
  cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);

  try
  {
    profile_ = pipe_.start(cfg_);

    // 低延迟关键：frames_queue_size = 1
    rs2::device dev = profile_.get_device();
    for (rs2::sensor s : dev.query_sensors())
    {
      if (s.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      {
        // 尽量让 timestamp 对齐到 host time（若设备支持）
        try
        {
          s.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        }
        catch (...)
        {
        }
      }
      if (s.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
      {
        s.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1.0f);
      }
    }

    // 取 color stream 内参
    auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics in = color_stream.get_intrinsics();

    // ViSP 相机模型（无畸变）
    cam.initPersProjWithoutDistortion(in.fx, in.fy, in.ppx, in.ppy);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("ObjectDetector: Failed to start RealSense pipeline: " << e.what());
    throw;
  }

  if (clockwise)
  {
    // 等价于“之前顺时针旋转像素”
    R_img << 0, -1, 0,
        1, 0, 0,
        0, 0, 1; // Rz(+90deg)
  }
  else
  {
    // 等价于“之前逆时针旋转像素”
    R_img << 0, 1, 0,
        -1, 0, 0,
        0, 0, 1; // Rz(-90deg)
  }

  // ----------------- 3. AprilTag 检测器配置 -----------------
  tag_detector.setAprilTagQuadDecimate(1);
  tag_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(8);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  // ----------------- 4. 位姿相关变量初始化 -----------------
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();

  R_i2c << 0.0141, -0.9999, 0.0078,
      -0.9997, -0.0142, -0.0181,
      0.0183, -0.0076, -0.9998;

  R_tagfix << 0, -1, 0,
      1, 0, 0,
      0, 0, 1;

  {
    Eigen::Quaterniond q_i2c(R_i2c);
    R_i2c = q_i2c.normalized().toRotationMatrix();
  }

  R_w2c = R_i2c;
  R_c2a = Eigen::Matrix3d::Identity();
  R_w2a = Eigen::Matrix3d::Identity();
  R_w2i = Eigen::Matrix3d::Identity();

  // ----------------- 5. ROS 通信 -----------------
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);

  odom_sub_ = nh_.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 200, // 队列建议大一点，方便插值
      &ObjectDetector::odomCallback, this,
      ros::TransportHints().tcpNoDelay());

  image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
}

ObjectDetector::~ObjectDetector()
{
  try
  {
    pipe_.stop();
  }
  catch (...)
  {
  }
}

void ObjectDetector::start()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(LOOP_HZ);
  while (ros::ok())
  {
    //double start_time = ros::Time::now().toSec();
    processImages();
    //double end_time = ros::Time::now().toSec();
    //cout << 1000 * (end_time - start_time) << endl;
    rate.sleep();
  }
}

void ObjectDetector::odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);

  // 你原本就用的“最新姿态”
  R_w2i = q.toRotationMatrix();

  // 新增：缓存带时间戳的姿态，给图像对齐用
  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    imu_buf_.push_back({msg->header.stamp, q});

    while (!imu_buf_.empty() &&
           (imu_buf_.back().t - imu_buf_.front().t).toSec() > imu_buf_span_sec_)
    {
      imu_buf_.pop_front();
    }
  }
}

bool ObjectDetector::getQuatAt(const ros::Time &t, Eigen::Quaterniond &q_out)
{
  std::lock_guard<std::mutex> lk(imu_mtx_);
  if (imu_buf_.size() < 2)
    return false;

  auto it = std::lower_bound(
      imu_buf_.begin(), imu_buf_.end(), t,
      [](const ImuSample &a, const ros::Time &tt)
      { return a.t < tt; });

  if (it == imu_buf_.begin())
  {
    q_out = it->q;
    return true;
  }
  if (it == imu_buf_.end())
  {
    q_out = imu_buf_.back().q;
    return true;
  }

  const auto &s1 = *(it - 1);
  const auto &s2 = *it;

  double denom = (s2.t - s1.t).toSec();
  if (denom <= 1e-6)
  {
    q_out = s2.q;
    return true;
  }

  double a = (t - s1.t).toSec() / denom;
  a = std::min(1.0, std::max(0.0, a));
  q_out = s1.q.slerp(a, s2.q);
  return true;
}

ros::Time ObjectDetector::frameToRosTime(const rs2::frame &f)
{
  ros::Time t;
  const double ts_ms = f.get_timestamp();
  const auto dom = f.get_frame_timestamp_domain();

  // 尽量直接用 epoch 域（SYSTEM_TIME / GLOBAL_TIME）
  if (dom == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME || dom == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME)
  {
    t.fromSec(ts_ms * 1e-3);
    return t;
  }

  // 次选：到达 host 的时间戳（如果可用）
  if (f.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
  {
    const double toa_ms = (double)f.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
    t.fromSec(toa_ms * 1e-3);
    return t;
  }

  // 兜底：一次性偏置把 device time 映射到 ros::Time::now
  if (!have_dev2ros_offset_)
  {
    dev2ros_offset_sec_ = ros::Time::now().toSec() - ts_ms * 1e-3;
    have_dev2ros_offset_ = true;
  }
  t.fromSec(ts_ms * 1e-3 + dev2ros_offset_sec_);
  return t;
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

// 你当前实现是“直接发布”（保持原样）
void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  if (!ros::ok())
    return;
  publishDetectionResult(ts, is_fault);
}

// 处理一轮：尽量拿“最新帧”
void ObjectDetector::processImages()
{
  ros::Time image_timestamp;
  bool fault_detected = false;

  try
  {
    // 1) 抽干队列：只保留最后一帧（最新帧）
    rs2::frameset fs, latest;
    bool has = false;
    while (pipe_.poll_for_frames(&fs))
    {
      latest = fs;
      has = true;
    }

    if (!has)
    {
      // 本周期没有新帧：发布 fault
      image_timestamp = ros::Time::now();
      fault_detected = true;
      lost_target = true;
      baseProcess(image_timestamp, fault_detected);
      return;
    }

    rs2::video_frame color = latest.get_color_frame();
    if (!color)
    {
      image_timestamp = ros::Time::now();
      fault_detected = true;
      lost_target = true;
      baseProcess(image_timestamp, fault_detected);
      return;
    }

    // 2) 图像时间戳（尽量真实）
    image_timestamp = frameToRosTime(color);

    // 3) RGB8 -> Gray
    const int w = color.get_width();
    const int h = color.get_height();
    cv::Mat rgb(h, w, CV_8UC3, (void *)color.get_data(), (size_t)color.get_stride_in_bytes());
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

    // 4) Gray -> vpImage<unsigned char>
    vpImage<unsigned char> I((unsigned int)h, (unsigned int)w);
    std::memcpy(I.bitmap, gray.data, (size_t)(w * h));

    // 5) 用图像时间戳插值 IMU 姿态，算 R_w2c
    Eigen::Quaterniond q_img;
    Eigen::Matrix3d R_w2i_img = R_w2i; // fallback：最新姿态
    if (getQuatAt(image_timestamp, q_img))
    {
      R_w2i_img = q_img.toRotationMatrix();
    }
    R_w2c = R_w2i_img * R_i2c;

    std_msgs::Header header;
    header.stamp = image_timestamp;
    sensor_msgs::ImagePtr img_msg =
        cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
    image_pub_.publish(img_msg);

    // 8) AprilTag 检测
    static std::vector<vpHomogeneousMatrix> cMo_vec;
    cMo_vec.clear();
    cMo_vec.reserve(1);

    tag_detector.detect(I, 0.0561, cam, cMo_vec);
    fault_detected = (cMo_vec.size() != 1);

    if (!fault_detected)
    {
      // 相机坐标系下 Tag 位置
      Position_before[0] = cMo_vec[0][0][3];
      Position_before[1] = cMo_vec[0][1][3];
      Position_before[2] = cMo_vec[0][2][3];
      Position_before = R_img * Position_before;
      Position_before += POS_OFFSET;

      // 世界系
      Position_after = R_w2c * Position_before;

      // 相机到 AprilTag 的旋转
      vpRotationMatrix R_c2a_vp = cMo_vec[0].getRotationMatrix();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          R_c2a(i, j) = R_c2a_vp[i][j];
      R_c2a = R_img * R_c2a;
      // 拼旋转链
      R_w2a = R_w2c * R_c2a * R_tagfix;
      desired_yaw = std::atan2(R_w2a(1, 0), R_w2a(0, 0));

      static int count_ = 0;
      if (++count_ % 30 == 0)
      {
        std::cout << "pos:     " << Position_after.transpose() << std::endl;
        std::cout << "yaw:     " << desired_yaw << std::endl;
        count_ = 0;
      }
      lost_target = false;
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
    lost_target = true;
  }

  baseProcess(image_timestamp, fault_detected);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;

  ObjectDetector detector(nh);
  detector.start();
  return 0;
}
