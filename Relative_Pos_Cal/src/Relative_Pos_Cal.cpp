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
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageFilter.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <cap_pic_from_cam_srv/CaptureImage.h>
#include <cap_pic_from_cam_srv/CameraCapture.h>
#include <condition_variable>

using namespace std;
using namespace std::chrono;

vpImage<unsigned char> I(480, 640); // 使用实际的高度和宽度初始化

vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag detector_(tagFamily);

std::mutex image_mutex; // 全局或成员变量互斥锁

int count_for_overtime = 0;

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                       q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

class ObjectDetector
{
public:
    ObjectDetector(ros::NodeHandle &nh)
        : nh_(nh), stop_thread(false), processing(false), desired_yaw(0), camera_cap("/dev/video0")
    {
        // 参数初始化
        opt_device = 0;
        poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY;
        quad_decimate = 1.0;
        nThreads = 1;
        display_tag = false;
        color_id = -1;
        thickness = 2;
        align_frame = false;
        display_off = true;

        Position_after = Eigen::Vector3d::Zero();

        R_c2a = Eigen::Matrix3d::Identity();
        R_i2c << 0.00698, -0.99997, 0.00279,
            -0.99988, -0.00694, 0.01416,
            -0.01414, -0.00289, -0.99990;
        R_w2a = Eigen::Matrix3d::Identity();
        R_w2c = R_i2c;

        // ROS topic和服务
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1);
        cv_image_pub = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 1, &ObjectDetector::odomCallback, this, ros::TransportHints().tcpNoDelay());
        timer = nh_.createTimer(ros::Duration(0.06), &ObjectDetector::timerCallback, this);
        worker_thread = thread(&ObjectDetector::processImages, this);

        // 初始化相机
        if (!camera_cap.init())
        {
            ROS_ERROR("Failed to initialize camera");
        }
    }

    ~ObjectDetector()
    {
        stop_thread = true; // 停止工作线程
        cv.notify_one();    // 通知等待的线程
        if (worker_thread.joinable())
        {
            worker_thread.join(); // 等待线程结束
        }
    }

    void start()
    {
        /*ros::AsyncSpinner spinner(2); // 使用2个线程
        spinner.start();*/
        ros::spin(); // 使用单线程
        ros::waitForShutdown();
    }

    void faultProcess(ros::Time &image_timestamp_getimg_)
    {
        desired_yaw = 0;
        lost_target = true;
        auto delay = ros::Duration(0.058) - (ros::Time::now() - image_timestamp_getimg_);
        if (delay > ros::Duration(0))
        {
            this_thread::sleep_for(std::chrono::milliseconds(int(delay.toSec() * 1000)));
        }
        else
        {
            count_for_overtime += 1;
        }
        publishDetectionResult(image_timestamp_getimg_);
        auto delay2 = ros::Time::now() - image_timestamp_getimg_;
        // cout << "image delay: " << 1000 * delay2.toSec() << endl;
        // cout << "viration times: " << count_for_overtime << endl;
        //  图像处理完成后，设置processing为false
        {
            std::lock_guard<std::mutex> lock(cv_mutex);
            processing = false;
        }
    }

    void normalProcess(ros::Time &image_timestamp_getimg_)
    {
        auto delay = ros::Duration(0.058) - (ros::Time::now() - image_timestamp_getimg_);
        if (delay > ros::Duration(0))
        {
            this_thread::sleep_for(std::chrono::milliseconds(int(delay.toSec() * 1000)));
        }
        else
        {
            count_for_overtime += 1;
        }
        publishDetectionResult(image_timestamp_getimg_);
        auto delay2 = ros::Time::now() - image_timestamp_getimg_;
        // cout << "image delay: " << 1000 * delay2.toSec() << endl;
        // cout << "viration times: " << count_for_overtime << endl;
        //  cout << desired_yaw << endl;
        //  图像处理完成后，设置processing为false
        {
            std::lock_guard<std::mutex> lock(cv_mutex);
            processing = false;
        }
    }

    void setupDetector()
    {
        detector_.setAprilTagQuadDecimate(quad_decimate);
        detector_.setAprilTagPoseEstimationMethod(poseEstimationMethod);
        detector_.setAprilTagNbThreads(nThreads);
        detector_.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
        detector_.setZAlignedWithCameraAxis(align_frame);
    }

    void initializeCameraParameters()
    {
        cam.initPersProjWithoutDistortion(426.44408, 427.70327, 344.18464, 255.63631);
        if (!intrinsic_file.empty() && !camera_name.empty())
        {
            parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
        }
        cout << cam << endl;
        cout << "poseEstimationMethod: " << poseEstimationMethod << endl;
        cout << "tagFamily: " << tagFamily << endl;
        cout << "nThreads : " << nThreads << endl;
        cout << "Z aligned: " << align_frame << endl;
        vpDisplay *d = NULL;
        if (!display_off)
        {
            d = new vpDisplayX(I);
        }
    }

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

    int opt_device;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod;
    double tagSize;
    float quad_decimate;
    int nThreads;
    string intrinsic_file;
    string camera_name;
    bool display_tag;
    int color_id;
    unsigned int thickness;
    bool align_frame;
    bool display_off;
    vpCameraParameters cam;
    vpXmlParserCamera parser;

    bool lost_target = true;
    map<int, double> tagSizes = {{0, 0.0254}, {1, 0.081}};
    double desired_yaw;
    Eigen::Vector3d Position_after;
    Eigen::Matrix3d R_c2a; // Rca 相机到apriltag
    Eigen::Matrix3d R_i2c; // Ric imu到相机
    Eigen::Matrix3d R_w2c; // Rwc 世界到相机
    Eigen::Matrix3d R_w2a; // Rwa 世界到apriltag
    std_msgs::Float64MultiArray point_;

    void publishDetectionResult(ros::Time &image_timestamp_)
    {
        point_.data = {Position_after(0), Position_after(1), Position_after(2),
                       image_timestamp_.toSec(), static_cast<double>(lost_target), desired_yaw};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void timerCallback(const ros::TimerEvent &)
    {
        {
            std::lock_guard<std::mutex> lock(cv_mutex);
            processing = true;
        }
        cv.notify_one();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        Eigen::Quaterniond q1(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        Eigen::Matrix3d R_w2c_temp = q1.toRotationMatrix() * R_i2c;
        {
            std::lock_guard<std::mutex> lock(data_mutex); // 加锁
            R_w2c = R_w2c_temp;                           // 保护对 R_w2c 的写操作
        } // 超出作用域后自动解锁
    }

    void processImages()
    {
        while (!stop_thread)
        {
            std::unique_lock<std::mutex> lock(cv_mutex);
            cv.wait(lock, [this]
                    { return processing.load() || stop_thread.load(); });
            if (stop_thread)
                break;
            lock.unlock(); // 解锁互斥锁
            cv::Mat distorted_image;
            Eigen::Matrix3d R_w2c_temp;
            {
                std::lock_guard<std::mutex> lock(data_mutex); // 加锁
                R_w2c_temp = R_w2c;                           // 保护对 R_w2c 的读操作
            } // 超出作用域后自动解锁
            ros::Time image_timestamp_getimg = ros::Time::now();
            try
            {
                camera_cap.captureImage(distorted_image);
                if (distorted_image.empty())
                {
                    ROS_ERROR("Captured image is empty.");
                    faultProcess(image_timestamp_getimg);
                    continue;
                }
                vpImageConvert::convert(distorted_image, I);
                // chrono::time_point<high_resolution_clock> image_timestamp_getimg_over = high_resolution_clock::now();
                // cout << "image get time: " << duration_cast<microseconds>(image_timestamp_getimg_over - image_timestamp_getimg).count() << endl;
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Exception: %s", e.what());
                faultProcess(image_timestamp_getimg);
                continue;
            }
            catch (...)
            {
                ROS_ERROR("Unknown exception occurred.");
                faultProcess(image_timestamp_getimg);
                continue;
            }

            // 对图像进行处理
            vector<vpHomogeneousMatrix> cMo_vec;
            vpHomogeneousMatrix pose_matrix;
            vpImageFilter::gaussianFilter(I, 2, 2);

            std_msgs::Header header;
            header.stamp = ros::Time::now();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", distorted_image).toImageMsg();
            cv_image_pub.publish(msg);

            detector_.detect(I);
            vector<int> ids = detector_.getTagsId();
            // 检测是否丢失目标
            if (ids.size() == 0)
            {
                cout << "lost target!!!!!!" << endl;
                faultProcess(image_timestamp_getimg);
                continue;
            }
            else
            {
                bool valid_id = true;
                for (const auto &id : ids)
                {
                    if (id != 0 && id != 1)
                    {
                        cout << "wrong detection!!!!!!" << endl;
                        valid_id = false;
                        break;
                    }
                }
                if (!valid_id)
                {
                    faultProcess(image_timestamp_getimg);
                    continue;
                }
                lost_target = false;
                bool pose_found = false;
                if (ids.size() == 2)
                {
                    int id = ids[1];
                    double tagSize = tagSizes[id];
                    vpHomogeneousMatrix cMo;
                    try
                    {
                        detector_.getPose(1, tagSize, cam, cMo);
                        cMo_vec.push_back(cMo);
                        pose_found = true;
                    }
                    catch (const std::exception &e)
                    {
                        cout << "Error in getPose: " << e.what() << endl;
                    }
                }
                else
                {
                    int id = ids[0];
                    double tagSize = tagSizes[id];
                    vpHomogeneousMatrix cMo;
                    try
                    {
                        detector_.getPose(0, tagSize, cam, cMo);
                        cMo_vec.push_back(cMo);
                        pose_found = true;
                    }
                    catch (const std::exception &e)
                    {
                        cout << "Error in getPose: " << e.what() << endl;
                    }
                }
                if (!pose_found || cMo_vec.empty())
                {
                    faultProcess(image_timestamp_getimg);
                    continue;
                }
                else
                {
                    pose_matrix = cMo_vec[0];
                    vpRotationMatrix R_c2a_vp;
                    pose_matrix.extract(R_c2a_vp);
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                        {
                            R_c2a(i * 3 + j) = R_c2a_vp[i][j];
                        }
                    }
                    R_w2a = R_w2c_temp * R_c2a;
                    if (ids.size() == 2 || ids[0] == 1)
                    {
                        Eigen::Vector3d t(0, -0.0584, 0);
                        t = R_c2a * t;
                        Position_after(0) = pose_matrix[0][3] - t[0];
                        Position_after(1) = pose_matrix[1][3] - t[1];
                        Position_after(2) = pose_matrix[2][3] - t[2];
                    }
                    else
                    {
                        Position_after(0) = pose_matrix[0][3];
                        Position_after(1) = pose_matrix[1][3];
                        Position_after(2) = pose_matrix[2][3];
                    }
                    Position_after(0) += 0.00125;
                    Position_after(1) -= 0.03655;
                    Position_after(2) += 0.02884; // 将其转换到imu飞控所在位置
                    Position_after = R_w2c_temp * Position_after;
                    static int loop = 0;
                    if (loop == 10)
                    {
                        cout << ids.size() << " " << ids[0] << endl;
                        cout << "Position_after: " << Position_after.transpose() << endl;
                        cout << count_for_overtime << endl;
                    }
                    loop++;
                    if (loop == 11)
                    {
                        loop = 0;
                    }

                    R_w2a = R_w2c_temp * R_c2a;
                    Eigen::Quaterniond q(R_w2a);
                    double yaw = fromQuaternion2yaw(q);
                    yaw = yaw + M_PI / 2;
                    desired_yaw = yaw;
                    normalProcess(image_timestamp_getimg);
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ObjectDetector detector(nh);
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--pose_method" && i + 1 < argc)
        {
            detector.poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc)
        {
            detector.tagSize = atof(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--camera_device" && i + 1 < argc)
        {
            detector.opt_device = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc)
        {
            detector.quad_decimate = (float)atof(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc)
        {
            detector.nThreads = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc)
        {
            detector.intrinsic_file = std::string(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc)
        {
            detector.camera_name = std::string(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--display_tag")
        {
            detector.display_tag = true;
        }
        else if (std::string(argv[i]) == "--display_off")
        {
            detector.display_off = true;
        }
        else if (std::string(argv[i]) == "--color" && i + 1 < argc)
        {
            detector.color_id = atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--thickness" && i + 1 < argc)
        {
            detector.thickness = (unsigned int)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc)
        {
            tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
        }
        else if (std::string(argv[i]) == "--z_aligned")
        {
            detector.align_frame = true;
        }
        else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
        {
            std::cout << "Usage: " << argv[0]
                      << " [--camera_device <camera device> (default: 0)]"
                      << " [--tag_size <tag_size in m> (default: 0.053)]"
                         " [--quad_decimate <quad_decimate> (default: 1)]"
                         " [--nthreads <nb> (default: 1)]"
                         " [--intrinsic <intrinsic file> (default: empty)]"
                         " [--camera_name <camera name>  (default: empty)]"
                         " [--pose_method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
                         " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
                         " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
                         " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
                         " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
                         " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
                         " [--display_tag] [--z_aligned]";
            std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
            std::cout << " [--help]" << std::endl;
            return EXIT_SUCCESS;
        }
    }
    detector.initializeCameraParameters();
    detector.setupDetector();
    detector.start();
    return 0;
}