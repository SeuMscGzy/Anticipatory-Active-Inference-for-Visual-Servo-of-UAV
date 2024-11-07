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
        nThreads = 4;
        display_tag = false;
        color_id = -1;
        thickness = 2;
        align_frame = false;
        display_off = true;

        Position_after = Eigen::Vector3d::Zero();

        R_c2a = Eigen::Matrix3d::Identity();
        R_i2c << -0.012299254251302336, -0.9997550667398611, -0.018399317184020714,
            -0.9988718281685636, 0.011440175054767979, 0.046088971413001924,
            -0.04586719128150388, 0.018945419570245543, -0.998767876856907;
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
        if (worker_thread.joinable())
        {
            worker_thread.join(); // 等待线程结束
        }
    }

    void start()
    {
        ros::AsyncSpinner spinner(2); // 使用2个线程
        spinner.start();
        ros::waitForShutdown();
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
        cam.initPersProjWithoutDistortion(537.9791181092193, 535.4888099676702, 427.0323350525588, 315.15525740107233);
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

    bool stop_thread;
    bool processing;

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

    void publishDetectionResult(chrono::time_point<high_resolution_clock> &image_timestamp_)
    {
        point_.data = {Position_after(0), Position_after(1), Position_after(2),
                       image_timestamp_.time_since_epoch().count(), static_cast<double>(lost_target), desired_yaw};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void timerCallback(const ros::TimerEvent &)
    {
        // 如果当前线程还在处理，则不执行新任务
        if (processing)
        {
            return;
        }
        else
        {
            processing = true; // 标志设置为正在处理
        }
        // cout << processing << endl;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        Eigen::Quaterniond q1(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        R_w2c = q1.toRotationMatrix() * R_i2c; // 世界系到相机系
    }

    void processImages()
    {
        while (!stop_thread)
        {
            if (processing)
            {
                cv::Mat distorted_image;
                Eigen::Matrix3d R_w2c_temp = R_w2c;
                chrono::time_point<high_resolution_clock> image_timestamp_getimg = high_resolution_clock::now();
                try
                {
                    camera_cap.captureImage(distorted_image);
                    cv::cvtColor(distorted_image, distorted_image, cv::COLOR_BGR2GRAY);
                    vpImageConvert::convert(distorted_image, I);
                    // chrono::time_point<high_resolution_clock> image_timestamp_getimg_over = high_resolution_clock::now();
                    // cout << "image get time: " << duration_cast<microseconds>(image_timestamp_getimg_over - image_timestamp_getimg).count() << endl;
                }
                catch (cv_bridge::Exception &e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    desired_yaw = 0;
                    lost_target = true;
                    processing = false;
                    auto delay = microseconds(58000) - duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                    if (delay.count() > 0)
                    {
                        this_thread::sleep_for(delay);
                    }
                    else
                    {
                        count_for_overtime += 1;
                    }
                    publishDetectionResult(image_timestamp_getimg);
                    auto delay2 = duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                    cout << delay2.count() << endl;
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
                if (ids.size() == 0)
                {
                    desired_yaw = 0;
                    lost_target = true;
                    cout << "lost!!!!!!" << endl;
                }
                else
                {
                    for (const auto &id : ids)
                    {
                        if (id != 0 && id != 1)
                        {
                            cout << "wrong detection!!!!!!" << endl;
                            desired_yaw = 0;
                            lost_target = true;
                            processing = false;
                            auto delay = microseconds(58000) - duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                            if (delay.count() > 0)
                            {
                                this_thread::sleep_for(delay);
                            }
                            else
                            {
                                count_for_overtime += 1;
                            }
                            publishDetectionResult(image_timestamp_getimg);
                            auto delay2 = duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                            cout << delay2.count() << endl;
                            continue;
                        }
                    }
                    lost_target = false;
                    for (size_t i = 0; i < ids.size(); i++)
                    {
                        int id = ids[i];
                        if (tagSizes.find(id) == tagSizes.end())
                        {
                            continue;
                        }
                        else
                        {
                            double tagSize = tagSizes[id];
                            vpHomogeneousMatrix cMo;
                            detector_.getPose(i, tagSize, cam, cMo);
                            cMo_vec.push_back(cMo);
                            break;
                        }
                    }
                    pose_matrix = cMo_vec[0];
                    vpRotationMatrix R_c2a_vp;
                    pose_matrix.extract(R_c2a_vp);
                    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_c2a_map(R_c2a.data());
                    R_c2a_map = Eigen::Matrix3d::Map(&R_c2a_vp[0][0]);
                    R_w2a = R_w2c_temp * R_c2a;
                    if (ids.size() == 2 || ids[0] == 1)
                    {
                        Eigen::Vector3d t(0, -0.0584, 0);
                        t = R_w2c_temp * t;
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
                    Position_after = R_w2c_temp * Position_after;
                    cout << "Position_after: " << Position_after.transpose() << endl;
                    Eigen::Quaterniond q(R_w2a);
                    double yaw = fromQuaternion2yaw(q);
                    yaw = yaw + M_PI / 2;
                    desired_yaw = yaw;
                }

                // chrono::time_point<high_resolution_clock> image_timestamp_temp3 = high_resolution_clock::now();
                //  cout << "image processing time: " << duration_cast<microseconds>(image_timestamp_temp3 - image_timestamp_temp2).count() << endl;
                auto delay = microseconds(58000) - duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                if (delay.count() > 0)
                {
                    this_thread::sleep_for(delay);
                }
                else
                {
                    count_for_overtime += 1;
                }
                publishDetectionResult(image_timestamp_getimg);
                processing = false;
                auto delay2 = duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_getimg);
                cout << "image delay: " << delay2.count() << endl;
                cout << "viration times: " << count_for_overtime << endl;
            }
            // 让线程稍作休息，避免空转
            this_thread::sleep_for(microseconds(1000)); // 休眠 1 毫秒
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