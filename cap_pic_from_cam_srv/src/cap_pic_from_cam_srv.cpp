#include <ros/ros.h>
#include "CameraCapture.h"
#include <cap_pic_from_cam_srv/CaptureImage.h> // 自定义服务类型
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <ros/callback_queue.h>

bool captureCallback(cap_pic_from_cam_srv::CaptureImage::Request &req,
                     cap_pic_from_cam_srv::CaptureImage::Response &res,
                     CameraCapture &camera)
{
    cv::Mat image;
    if (camera.captureImage(image))
    {
        // 将图像转换为ROS消息
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

        // 设置响应
        res.image = *msg;
        res.success = true;
        res.message = "Image captured successfully";
    }
    else
    {
        res.success = false;
        res.message = "Failed to capture image";
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_service_node");
    ros::NodeHandle nh;

    // 从参数服务器获取相机地址，默认值为 "0"
    std::string camera_address;
    nh.param<std::string>("camera_address", camera_address, "0");

    // 创建相机捕获对象
    CameraCapture camera(camera_address);

    // 初始化相机
    if (!camera.init())
    {
        ROS_ERROR("Failed to initialize camera");
        return -1;
    }

    // 创建服务服务器
    ros::ServiceServer service = nh.advertiseService<cap_pic_from_cam_srv::CaptureImage::Request, cap_pic_from_cam_srv::CaptureImage::Response>(
        "capture_image", boost::bind(captureCallback, _1, _2, boost::ref(camera)));

    // 创建异步 spinner，指定线程数（例如 2 个线程）
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    camera.release();
    return 0;
}
