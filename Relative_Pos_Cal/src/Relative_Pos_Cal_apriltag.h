#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <librealsense2/rs.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorAprilTag.h>

#include <deque>
#include <mutex>
#include <algorithm>

class ObjectDetector
{
public:
    explicit ObjectDetector(ros::NodeHandle &nh);
    ~ObjectDetector();

    void start();

private:
    // ===== ROS =====
    ros::NodeHandle nh_;
    ros::Publisher point_pub_;
    ros::Publisher image_pub_;
    ros::Subscriber odom_sub_;

    // ===== RealSense (native librealsense) =====
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::pipeline_profile profile_;
    int width_ = 1280;
    int height_ = 720;
    int fps_ = 30;
    /*int width_ = 640;
    int height_ = 480;
    int fps_ = 60;*/

    bool have_dev2ros_offset_ = false;
    double dev2ros_offset_sec_ = 0.0;

    // ===== ViSP AprilTag =====
    vpDetectorAprilTag tag_detector;
    vpCameraParameters cam;     // original
    vpCameraParameters cam_rot; // after 90 deg rotation

    // ===== State =====
    bool lost_target;
    double desired_yaw;
    bool clockwise;

    Eigen::Vector3d Position_before;
    Eigen::Vector3d Position_after;

    Eigen::Matrix3d R_i2c;
    Eigen::Matrix3d R_tagfix;

    Eigen::Matrix3d R_w2c;
    Eigen::Matrix3d R_c2a;
    Eigen::Matrix3d R_w2a;
    Eigen::Matrix3d R_w2i;
    Eigen::Matrix3d R_img;

    // ===== IMU buffer for time alignment =====
    struct ImuSample
    {
        ros::Time t;
        Eigen::Quaterniond q;
    };
    std::mutex imu_mtx_;
    std::deque<ImuSample> imu_buf_;
    double imu_buf_span_sec_ = 1.0;

private:
    void odomCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void publishDetectionResult(ros::Time &ts, bool is_fault);
    void baseProcess(ros::Time ts, bool is_fault);
    void processImages(); // process one iteration (one newest frame if available)

    bool getQuatAt(const ros::Time &t, Eigen::Quaterniond &q_out);
    ros::Time frameToRosTime(const rs2::frame &f);
};
