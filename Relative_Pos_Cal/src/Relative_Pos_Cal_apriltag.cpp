#include "Relative_Pos_Cal_apriltag.h"

using namespace std;
using namespace std::chrono;
template <typename T>
void flipImage(vpImage<T> &src, vpImage<T> &dst, int flipCode)
{
  dst.resize(src.getHeight(), src.getWidth());

  if (flipCode == 0)
  {
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[src.getHeight() - i - 1][j] = src[i][j];
      }
    }
  }
  else if (flipCode > 0)
  {
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[i][src.getWidth() - j - 1] = src[i][j];
      }
    }
  }
  else
  {
    for (unsigned int i = 0; i < src.getHeight(); ++i)
    {
      for (unsigned int j = 0; j < src.getWidth(); ++j)
      {
        dst[src.getHeight() - i - 1][src.getWidth() - j - 1] = src[i][j];
      }
    }
  }
}

constexpr double PROCESSING_LATENCY = 0.04; // 40ms处理延迟
const Eigen::Vector3d POS_OFFSET{0.00625, -0.08892, 0.06430};
ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh), stop_thread(false), processing(false),
      lost_target(true), desired_yaw(0)
{
  config.disable_stream(RS2_STREAM_DEPTH);
  config.disable_stream(RS2_STREAM_INFRARED);
  config.enable_stream(RS2_STREAM_COLOR, 480, 270, RS2_FORMAT_RGBA8, 30);
  g.open(config);
  cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  tag_detector.setAprilTagQuadDecimate(1.5);
  tag_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();
  R_i2c << 0.00400, -0.03121, 0.99950,
      -0.99996, 0.00749, 0.00423,
      -0.00762, -0.99948, -0.03118;
  R_w2c = R_i2c;
  R_c2a = Eigen::Matrix3d::Identity();
  R_w2a = Eigen::Matrix3d::Identity();

  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/imu/data", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
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
      /*vpImage<unsigned char> flippedImage;
      flipImage(I, flippedImage, -1);
      I = flippedImage;*/
      cv::Mat imageMat;
      vpImageConvert::convert(I, imageMat);
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", imageMat).toImageMsg();
      image_pub.publish(msg);
      std::vector<vpHomogeneousMatrix> cMo_vec;
      tag_detector.detect(I, 0.0795, cam, cMo_vec);
      ros::Time detection_time = ros::Time::now();
      cout << "Detection time: " << detection_time.toSec() - image_timestamp.toSec() << " seconds." << endl;
      bool fault_detected = (cMo_vec.size() != 1);

      if (!fault_detected)
      {
        Position_before[0] = cMo_vec[0][0][3];
        Position_before[1] = cMo_vec[0][1][3];
        Position_before[2] = cMo_vec[0][2][3];

        vpRotationMatrix R_c2a_vp = cMo_vec[0].getRotationMatrix();
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            R_c2a(i, j) = R_c2a_vp[i][j];

        R_w2a = R_w2c * R_c2a;
        Eigen::Vector3d euler = R_w2a.eulerAngles(2, 1, 0);
        double yaw = euler[0];
        Position_before += POS_OFFSET;
        Position_after = R_w2c * Position_before;

        desired_yaw = yaw + M_PI_2;
        if (desired_yaw > M_PI)
          desired_yaw -= 2 * M_PI;

        lost_target = false;

        //cout << "Position_after: " << Position_after.transpose() << endl;
        //cout << "Desired yaw: " << desired_yaw << endl;
      }
      else
      {
        //ROS_INFO_STREAM("No tag detected or multiple tags detected.");
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
