#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>

class CameraCapture
{
public:
    CameraCapture(const std::string &device_address = "0");
    ~CameraCapture();

    bool init();
    bool captureImage(cv::Mat &image);
    void release();

private:
    std::string device_address_;
    cv::VideoCapture cap_;
    bool is_initialized_;
};

#endif // CAMERA_CAPTURE_H
