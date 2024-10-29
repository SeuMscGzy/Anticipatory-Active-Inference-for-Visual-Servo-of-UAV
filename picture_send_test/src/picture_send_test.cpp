#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "blank_image_publisher");
    ros::NodeHandle nh;

    // 创建图像发布者
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);

    // 创建一张空白的黑色图像（假设分辨率为640x480）
    cv::Mat blank_image = cv::Mat::zeros(600, 800, CV_8UC3);  // 8位无符号3通道图像

    // 设置发布频率为60Hz
    ros::Rate loop_rate(60);

    while (ros::ok()) {
        // 将OpenCV的图像转换为ROS图像消息
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8";  // 图像编码
        cv_image.image = blank_image;

        // 发布图像消息
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        image_pub.publish(ros_image);

        ROS_INFO_ONCE("Blank image publishing at 60Hz to /usb_cam/image_raw");

        // 维持发布频率
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
