#include "cap_pic_from_cam_srv/CameraCapture.h"

CameraCapture::CameraCapture(const std::string &device_address)
    : device_address_(device_address), is_initialized_(false)
{
}

CameraCapture::~CameraCapture()
{
    release();
}

bool CameraCapture::init()
{
    // 尝试将设备地址解析为整数设备ID
    bool is_device_id = true;
    int device_id = 0;
    try
    {
        device_id = std::stoi(device_address_);
    }
    catch (const std::invalid_argument &)
    {
        is_device_id = false;
    }

    // 根据设备地址类型打开相机
    if (is_device_id)
    {
        cap_.open(device_id, cv::CAP_V4L2);
        ROS_INFO("Opening camera with device ID: %d", device_id);
    }
    else
    {
        cap_.open(device_address_, cv::CAP_V4L2);
        ROS_INFO("Opening camera with device address: %s", device_address_.c_str());
    }

    if (!cap_.isOpened())
    {
        ROS_ERROR("Cannot open camera with address %s", device_address_.c_str());
        return false;
    }

    // 设置相机参数（可选）
    cap_.set(cv::CAP_PROP_CONVERT_RGB, false);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    // cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    int format = static_cast<int>(cap_.get(cv::CAP_PROP_FORMAT));
    ROS_INFO("Camera format code: %d (0x%X)", format, format);

    is_initialized_ = true;
    return true;
}

bool CameraCapture::captureImage(cv::Mat &image)
{
    if (!is_initialized_)
        return false;
    cv::Mat raw_frame;
    if (!cap_.read(raw_frame))
        return false;
    // YUYV格式为2通道，直接提取Y通道
    if (raw_frame.type() == CV_8UC2)
    {
        cv::cvtColor(raw_frame, image, cv::COLOR_YUV2GRAY_YUYV);
        return true;
    }
    else
    {
        ROS_ERROR("Unsupported format: %d channels", raw_frame.channels());
        return false;
    }
}

void CameraCapture::release()
{
    if (is_initialized_)
    {
        cap_.release();
        is_initialized_ = false;
    }
}
