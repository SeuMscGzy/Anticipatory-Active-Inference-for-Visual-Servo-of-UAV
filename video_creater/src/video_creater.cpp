#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

class ImageToVideo
{
public:
    ImageToVideo()
    {
        // 初始化节点，订阅图像话题
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("/camera/image", 1, &ImageToVideo::imageCallback, this);

        // 初始化视频编写器
        video_writer = cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(640, 480));
    }
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV图像格式
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            if (!frame.empty())
            {
                video_writer.write(frame);
                ROS_INFO("Image received and written to video");
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    ros::NodeHandle nh;
    image_transport::Subscriber sub;
    cv::VideoWriter video_writer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_video_node");
    ImageToVideo itv;
    ros::spin();
    return 0;
}
