#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "ros/time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
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
#include <librealsense2/rs.hpp>
#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageFilter.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <librealsense2/rs_advanced_mode.hpp> // 注意要包含这个头
using namespace std;
using namespace std::chrono;

class ObjectDetector
{
public:
    ros::NodeHandle nh_;
    ros::Publisher point_pub_;
    ros::Publisher cv_image_pub;
    ros::Subscriber odom_sub_;
    ros::Publisher image_pub;
    ros::Timer timer;
    thread worker_thread;

    std::atomic<bool> stop_thread;
    std::atomic<bool> processing;
    std::mutex data_mutex;
    std::mutex R_mutex;
    std::condition_variable cv;

    bool lost_target;
    double desired_yaw;
    Eigen::Vector3d Position_before, Position_after;
    Eigen::Matrix3d R_c2a;
    Eigen::Matrix3d R_i2c;
    Eigen::Matrix3d R_w2c;
    Eigen::Matrix3d R_w2a;
    bool clockwise = true;

    vpRealSense2 g;
    rs2::config config;
    vpCameraParameters cam;
    vpCameraParameters cam_rot;
    vpDetectorAprilTag tag_detector;

    std_msgs::Float64MultiArray point_;
    ObjectDetector(ros::NodeHandle &nh);
    ~ObjectDetector();
    void start();

private:
    void timerCallback(const ros::TimerEvent &);
    void odomCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void baseProcess(ros::Time ts, bool is_fault);
    void publishDetectionResult(ros::Time &ts, bool is_fault);
    double fromQuaternion2yaw(const Eigen::Quaterniond &q);
    void processImages();
};

#endif // OBJECT_DETECTOR_H