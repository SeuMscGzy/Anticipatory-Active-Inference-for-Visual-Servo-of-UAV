#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "ros/time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <cap_pic_from_cam_srv/CameraCapture.h>
#include <cap_pic_from_cam_srv/CaptureImage.h>
#include <chrono>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <unordered_map>
#include <vector>
#include <stdexcept>

using namespace std;
using namespace std::chrono;

class ObjectDetector
{
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
    std::mutex data_mutex;
    std::condition_variable cv;

    bool lost_target;
    double desired_yaw;
    Eigen::Vector3d Position_before, Position_after;
    Eigen::Matrix3d R_c2a;
    Eigen::Matrix3d R_i2c;
    Eigen::Matrix3d R_w2c;
    Eigen::Matrix3d R_w2a;

    // 将 cv::Mat rvec 和 tvec 作为类成员变量用于复用内存
    cv::Mat rvec_;
    cv::Mat tvec_;

    std_msgs::Float64MultiArray point_;
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    cv::Mat cameraMatrix, distCoeffs;
    unordered_map<int, double> tag_sizes;
    unordered_map<int, vector<cv::Point3d>> cached_obj_pts_;

    // 构造函数与析构函数
    ObjectDetector(ros::NodeHandle &nh);
    ~ObjectDetector();
    void start();

private:
    void timerCallback(const ros::TimerEvent &);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void processSingleTag(apriltag_detection_t *det, Eigen::Matrix3d &R_c2a, Eigen::Vector3d &position);
    void baseProcess(ros::Time ts, bool is_fault);
    void publishDetectionResult(ros::Time &ts, bool is_fault);
    double fromQuaternion2yaw(const Eigen::Quaterniond &q);
    void processImages();
};

#endif // OBJECT_DETECTOR_H
