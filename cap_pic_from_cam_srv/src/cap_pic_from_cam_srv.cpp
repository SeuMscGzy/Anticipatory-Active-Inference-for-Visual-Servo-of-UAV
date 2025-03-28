#include <ros/ros.h>
#include <cap_pic_from_cam_srv/CameraCapture.h>
#include <cap_pic_from_cam_srv/CaptureImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

bool captureCallback(
    cap_pic_from_cam_srv::CaptureImage::Request &req,
    cap_pic_from_cam_srv::CaptureImage::Response &res,
    CameraCapture &camera)
{
    cv::Mat image;
    if (camera.captureImage(image))
    {
        // 确保图像已经是单通道灰度（YUYV的Y通道已提取）
        if (image.channels() == 1)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                                            std_msgs::Header(), "mono8", image)
                                            .toImageMsg();
            res.image = *msg;
            res.success = true;
            res.message = "Image captured successfully";
        }
        else
        {
            ROS_ERROR("Expected grayscale image, but got %d channels", image.channels());
            res.success = false;
            res.message = "Camera output format error";
        }
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

    std::string camera_address;
    nh.param<std::string>("camera_address", camera_address, "0");

    // 初始化相机并强制设置为YUYV格式
    CameraCapture camera(camera_address);
    if (!camera.init())
    {
        ROS_ERROR("Failed to initialize camera");
        return -1;
    }

    ros::ServiceServer service = nh.advertiseService<cap_pic_from_cam_srv::CaptureImage::Request, cap_pic_from_cam_srv::CaptureImage::Response>(
        "capture_image",
        [&camera](auto &req, auto &res)
        { return captureCallback(req, res, camera); });

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    camera.release();
    return 0;
}
