#include "ros/time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <cap_pic_from_cam_srv/CameraCapture.h>
#include <cap_pic_from_cam_srv/CaptureImage.h>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>

using namespace std;
using namespace std::chrono;

std::mutex image_mutex; // 全局或成员变量互斥锁

int count_for_overtime = 0;

double fromQuaternion2yaw(Eigen::Quaterniond q) {
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

class ObjectDetector {
public:
  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  ros::Publisher cv_image_pub;
  ros::Subscriber odom_sub_;
  ros::Timer timer;

  CameraCapture camera_cap;

  thread worker_thread;

  std::atomic<bool> stop_thread;
  std::atomic<bool> processing;

  std::mutex data_mutex; // 用于保护共享数据 R_w2c
  // 新增成员变量
  std::condition_variable cv;
  std::mutex cv_mutex;

  bool lost_target = true;
  double desired_yaw;
  Eigen::Vector3d Position_before, Position_after;
  Eigen::Matrix3d R_c2a; // Rca 相机到apriltag
  Eigen::Matrix3d R_i2c; // Ric imu到相机
  Eigen::Matrix3d R_w2c; // Rwc 世界到相机
  Eigen::Matrix3d R_w2a; // Rwa 世界到apriltag

  std_msgs::Float64MultiArray point_;
  apriltag_detector_t *td;
  apriltag_family_t *tf;
  cv::Mat cameraMatrix, distCoeffs;
  std::unordered_map<int, double> tag_sizes;

  ObjectDetector(ros::NodeHandle &nh)
      : nh_(nh), stop_thread(false), processing(false), desired_yaw(0),
        camera_cap("/dev/video0") {
    // 初始化AprilTag检测器
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    // td->quad_decimate = 1.0; // 降采样比例
    td->nthreads = 4; // 并行线程数

    // 配置参数（示例值，需根据实际相机标定）
    cameraMatrix = (cv::Mat_<double>(3, 3) << 426.44408, 0, 344.18464, 0,
                    427.70327, 255.63631, 0, 0, 1);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    tag_sizes = {{0, 0.0254}, // ID 0对应标签尺寸
                 {1, 0.081}}; // ID 1对应标签尺寸

    Position_before = Eigen::Vector3d::Zero();
    Position_after = Eigen::Vector3d::Zero();
    R_c2a = Eigen::Matrix3d::Identity();
    R_i2c << 0.00698, -0.99997, 0.00279, -0.99988, -0.00694, 0.01416, -0.01414,
        -0.00289, -0.99990;
    R_w2a = Eigen::Matrix3d::Identity();
    R_w2c = R_i2c;

    // ROS topic和服务
    point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/point_with_fixed_delay", 1);
    cv_image_pub = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/vins_fusion/imu_propagate", 1, &ObjectDetector::odomCallback, this,
        ros::TransportHints().tcpNoDelay());
    timer = nh_.createTimer(ros::Duration(0.06), &ObjectDetector::timerCallback,
                            this);
    worker_thread = thread(&ObjectDetector::processImages, this);

    // 初始化相机
    if (!camera_cap.init()) {
      ROS_ERROR("Failed to initialize camera");
    }
  }

  ~ObjectDetector() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    stop_thread = true; // 停止工作线程
    cv.notify_one();    // 通知等待的线程
    if (worker_thread.joinable()) {
      worker_thread.join(); // 等待线程结束
    }
  }

  void start() {
    /*ros::AsyncSpinner spinner(2); // 使用2个线程
    spinner.start();*/
    ros::spin(); // 使用单线程
    ros::waitForShutdown();
  }

  void faultProcess(ros::Time &image_timestamp_getimg_) {
    desired_yaw = 0;
    lost_target = true;
    auto delay =
        ros::Duration(0.058) - (ros::Time::now() - image_timestamp_getimg_);
    if (delay > ros::Duration(0)) {
      this_thread::sleep_for(
          std::chrono::milliseconds(int(delay.toSec() * 1000)));
    } else {
      count_for_overtime += 1;
    }
    publishDetectionResult(image_timestamp_getimg_);
    auto delay2 = ros::Time::now() - image_timestamp_getimg_;
    {
      std::lock_guard<std::mutex> lock(cv_mutex);
      processing = false;
    }
  }

  void normalProcess(ros::Time &image_timestamp_getimg_) {
    auto delay =
        ros::Duration(0.058) - (ros::Time::now() - image_timestamp_getimg_);
    if (delay > ros::Duration(0)) {
      this_thread::sleep_for(
          std::chrono::milliseconds(int(delay.toSec() * 1000)));
    } else {
      count_for_overtime += 1;
    }
    publishDetectionResult(image_timestamp_getimg_);
    auto delay2 = ros::Time::now() - image_timestamp_getimg_;
    {
      std::lock_guard<std::mutex> lock(cv_mutex);
      processing = false;
    }
  }

  void publishDetectionResult(ros::Time &image_timestamp_) {
    point_.data = {Position_after(0),
                   Position_after(1),
                   Position_after(2),
                   image_timestamp_.toSec(),
                   static_cast<double>(lost_target),
                   desired_yaw};
    point_pub_.publish(point_);
    point_.data.clear();
  }

  void timerCallback(const ros::TimerEvent &) {
    {
      std::lock_guard<std::mutex> lock(cv_mutex);
      processing = true;
    }
    cv.notify_one();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    Eigen::Quaterniond q1(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix3d R_w2c_temp = q1.toRotationMatrix() * R_i2c;
    {
      std::lock_guard<std::mutex> lock(data_mutex); // 加锁
      R_w2c = R_w2c_temp; // 保护对 R_w2c 的写操作
    }                     // 超出作用域后自动解锁
  }

  // 独立位姿计算函数
  void processSingleTag(apriltag_detection_t *det, Eigen::Matrix3d &R_c2a,
                        Eigen::Vector3d &position) {
    // 构建3D-2D对应点
    auto it = tag_sizes.find(det->id);
    double tag_size = it->second;
    vector<cv::Point3d> obj_pts = {
        {-tag_size / 2, tag_size / 2, 0}, // 左上角（对应 det->p[0]）
        {tag_size / 2, tag_size / 2, 0},  // 右上角（对应 det->p[1]）
        {tag_size / 2, -tag_size / 2, 0}, // 右下角（对应 det->p[2]）
        {-tag_size / 2, -tag_size / 2, 0} // 左下角（对应 det->p[3]）
    };
    vector<cv::Point2d> img_pts = {{det->p[0][0], det->p[0][1]},
                                   {det->p[1][0], det->p[1][1]},
                                   {det->p[2][0], det->p[2][1]},
                                   {det->p[3][0], det->p[3][1]}};
    // 解算位姿
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec, tvec,
                      false, cv::SOLVEPNP_IPPE_SQUARE)) {
      throw std::runtime_error("Pose estimation failed");
    }
    // 转换为Eigen矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::cv2eigen(R, R_c2a);
    position = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1),
                               tvec.at<double>(2));
  }

  void processImages() {
    while (!stop_thread) {
      std::unique_lock<std::mutex> lock(cv_mutex);
      cv.wait(lock, [this] { return processing.load() || stop_thread.load(); });
      if (stop_thread)
        break;
      lock.unlock(); // 解锁互斥锁
      cv::Mat distorted_image;
      zarray_t *detections;
      Eigen::Matrix3d R_w2c_temp;
      {
        std::lock_guard<std::mutex> lock(data_mutex); // 加锁
        R_w2c_temp = R_w2c; // 保护对 R_w2c 的读操作
      }                     // 超出作用域后自动解锁
      ros::Time image_timestamp_getimg = ros::Time::now();
      try {
        camera_cap.captureImage(distorted_image);
        if (distorted_image.empty()) {
          ROS_ERROR("Captured image is empty.");
          faultProcess(image_timestamp_getimg);
          continue;
        }

        // 转换为灰度图（如果是彩色图）
        if (distorted_image.channels() == 3) {
          cv::cvtColor(distorted_image, distorted_image, cv::COLOR_BGR2GRAY);
        }

        // 创建一个 image_u8_t 结构来包装 cv::Mat 的数据
        image_u8_t apriltag_image = {.width = distorted_image.cols,
                                     .height = distorted_image.rows,
                                     .stride = distorted_image.step,
                                     .buf = distorted_image.data};
        zarray_t *detections_temp =
            apriltag_detector_detect(td, &apriltag_image);
        detections = detections_temp;
      } catch (const std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
        faultProcess(image_timestamp_getimg);
        continue;
      } catch (...) {
        ROS_ERROR("Unknown exception occurred.");
        faultProcess(image_timestamp_getimg);
        continue;
      }

      /*std_msgs::Header header;
      header.stamp = ros::Time::now();
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(header, "mono8", distorted_image).toImageMsg();
      cv_image_pub.publish(msg);*/

      // 检测AprilTag
      cout << zarray_size(detections) << endl;
      Eigen::Matrix3d R_c2a;
      Eigen::Vector3d position_before;
      // 单次遍历，即时处理优先级逻辑
      bool has_invalid_tag = false;
      int selected_index = -1;
      bool found_tag1 = false;
      // 第一遍遍历：确定需要处理的标签索引
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // 有效性检查
        if (det->id != 0 && det->id != 1) {
          has_invalid_tag = true;
          break;
        } else {
          has_invalid_tag = false;
          lost_target = false;
          // 优先记录标签1的位置
          if (det->id == 1) {
            selected_index = i;
            found_tag1 = true;
            break; // 找到第一个标签1立即终止循环
          }
          // 记录第一个有效标签（只有当未找到标签1时才更新）
          if (!found_tag1 && selected_index == -1) {
            selected_index = i;
          }
        }
      }
      if (has_invalid_tag) {
        cout << "fualt detection!!!!!!" << endl;
        faultProcess(image_timestamp_getimg);
        continue;
      }
      // 处理检测结果（保持原有逻辑）
      if (selected_index == -1) {
        cout << "lost target!!!!!!" << endl;
        lost_target = true;
        faultProcess(image_timestamp_getimg);
        continue;
      } else {
        apriltag_detection_t *det;
        zarray_get(detections, selected_index, &det);
        processSingleTag(det, R_c2a, position_before);
      }
      apriltag_detections_destroy(detections);
      // 坐标系转换（保持原有补偿逻辑）
      Eigen::Vector3d t(0, -0.0584, 0);
      R_w2a = R_w2c_temp * R_c2a;
      if (found_tag1 == true) {
        t = R_c2a * t;
        position_before -= t;
      }
      // 应用固定补偿
      position_before.x() += 0.00125;
      position_before.y() -= 0.03655;
      position_before.z() += 0.02884;
      // 转换到世界坐标系
      Position_after = R_w2c_temp * position_before;
      //ros::Time image_over_time = ros::Time::now();
      //cout<<"time cost:"<< 1000*(image_over_time.toSec() - image_timestamp_getimg.toSec())<<"ms"<<endl;
      //cout << "Position_after: " << Position_after.transpose() << endl;
      // 计算偏航角
      Eigen::Quaterniond q(R_w2a);
      double yaw = fromQuaternion2yaw(q);
      yaw += M_PI / 2;
      // 更新状态
      desired_yaw = yaw;
      lost_target = false;
      // 后续处理
      normalProcess(image_timestamp_getimg);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ObjectDetector detector(nh);
  detector.start();
  return 0;
}