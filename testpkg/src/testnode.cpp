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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
int argc_;
char **argv_;
using namespace std;
using namespace std::chrono;

vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag detector_(tagFamily);

class ObjectDetector
{
public:
    ObjectDetector(ros::NodeHandle &nh)
        : image_transport(nh), nh_(nh), I_read(600, 800), I_write(600, 800), stop_thread(false), processing(false)
    {
        // 参数初始化
        opt_device = 0;
        poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
        tagSize = 0.053;
        quad_decimate = 1.0;
        nThreads = 2;
        display_tag = false;
        color_id = -1;
        thickness = 2;
        align_frame = false;
        display_off = true;

        // parseCommandLineArgs();
        initializeCameraParameters();
        setupDetector();

        // ROS topic和服务
        pbvs_publisher = nh_.advertise<std_msgs::Float64MultiArray>("/object_pose", 1);
        cv_image_pub = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
        R_publisher = nh_.advertise<std_msgs::Float64MultiArray>("/R_data", 1);
        imgsub = nh_.subscribe("/usb_cam/image_raw", 1, &ObjectDetector::imageCallback, this, ros::TransportHints().tcpNoDelay());
        timer = nh_.createTimer(ros::Duration(0.05), &ObjectDetector::timerCallback, this);
        worker_thread = std::thread(&ObjectDetector::processImages, this);
        ros_pbvs_msg.data.resize(6, 0.0);
        R_msg.data.resize(9);
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
        ros::AsyncSpinner spinner(2); // 使用两个线程
        spinner.start();
        ros::waitForShutdown();
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport;
    ros::Publisher pbvs_publisher;
    ros::Publisher cv_image_pub;
    ros::Publisher R_publisher;
    ros::Subscriber imgsub;
    ros::Timer timer;

    std::chrono::time_point<std::chrono::high_resolution_clock> image_timestamp;
    vpImage<unsigned char> I_read;  // 用于读取的缓冲
    vpImage<unsigned char> I_write; // 用于写入的缓冲
    std::mutex buffer_mutex;
    std::thread worker_thread;
    std::atomic<bool> stop_thread;
    std::atomic<bool> processing;
    std_msgs::Float64MultiArray ros_pbvs_msg;
    std_msgs::Float64MultiArray R_msg;

    int opt_device;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod;
    double tagSize;
    float quad_decimate;
    int nThreads;
    std::string intrinsic_file;
    std::string camera_name;
    bool display_tag;
    int color_id;
    unsigned int thickness;
    bool align_frame;
    bool display_off;
    vpCameraParameters cam;
    vpXmlParserCamera parser;

    void swapBuffers()
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        std::swap(I_read, I_write);
    }

    int parseCommandLineArgs()
    {
        for (int i = 1; i < argc_; i++)
        {
            if (std::string(argv_[i]) == "--pose_method" && i + 1 < argc_)
            {
                poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--tag_size" && i + 1 < argc_)
            {
                tagSize = atof(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--camera_device" && i + 1 < argc_)
            {
                opt_device = atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--quad_decimate" && i + 1 < argc_)
            {
                quad_decimate = (float)atof(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--nthreads" && i + 1 < argc_)
            {
                nThreads = atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--intrinsic" && i + 1 < argc_)
            {
                intrinsic_file = std::string(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--camera_name" && i + 1 < argc_)
            {
                camera_name = std::string(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--display_tag")
            {
                display_tag = true;
            }
            else if (std::string(argv_[i]) == "--display_off")
            {
                display_off = true;
            }
            else if (std::string(argv_[i]) == "--color" && i + 1 < argc_)
            {
                color_id = atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--thickness" && i + 1 < argc_)
            {
                thickness = (unsigned int)atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--tag_family" && i + 1 < argc_)
            {
                tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv_[i + 1]);
            }
            else if (std::string(argv_[i]) == "--z_aligned")
            {
                align_frame = true;
            }
            else if (std::string(argv_[i]) == "--help" || std::string(argv_[i]) == "-h")
            {
                std::cout << "Usage: " << argv_[0]
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
    }

    void initializeCameraParameters()
    {
        cam.initPersProjWithoutDistortion(537.9791181092193, 535.4888099676702, 427.0323350525588, 315.15525740107233);
        if (!intrinsic_file.empty() && !camera_name.empty())
        {
            parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
        }
        std::cout << cam << std::endl;
        std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
        std::cout << "tagFamily: " << tagFamily << std::endl;
        std::cout << "nThreads : " << nThreads << std::endl;
        std::cout << "Z aligned: " << align_frame << std::endl;
        vpDisplay *d = NULL;
        if (!display_off)
        {
            d = new vpDisplayX(I_read);
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

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            image_timestamp = high_resolution_clock::now();
            cv::Mat distorted_image = cv_bridge::toCvShare(msg, "mono8")->image;
            if (I_write.getWidth() == 0 || I_write.getHeight() == 0)
            {
                I_write.resize(distorted_image.rows, distorted_image.cols);
            }
            vpImageConvert::convert(distorted_image, I_write);
            swapBuffers();
            cout << "image callback" << endl;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
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
        //cout << processing << endl;
    }

    void processImages()
    {
        while (!stop_thread)
        {
            if (processing)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> image_timestamp_temp = image_timestamp;
                std::vector<vpHomogeneousMatrix> cMo_vec;
                vpHomogeneousMatrix pose_matrix;
                vpImageFilter::gaussianFilter(I_read, 3, 3);
                cv::Mat imageMat;
                vpImageConvert::convert(I_read, imageMat);

                std_msgs::Header header;
                header.stamp = ros::Time::now();
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", imageMat).toImageMsg();
                cv_image_pub.publish(msg);

                detector_.detect(I_read);
                std::map<int, double> tagSizes = {{0, 0.0254}, {1, 0.081}};
                std::vector<int> ids = detector_.getTagsId();
                if (ids.size() == 0)
                {
                    // 没有检测到标签
                }
                else
                {
                    for (size_t i = 0; i < ids.size(); i++)
                    {
                        int id = ids[i];
                        if (tagSizes.find(id) == tagSizes.end())
                        {
                            continue;
                        }
                        double tagSize = tagSizes[id];
                        vpHomogeneousMatrix cMo;
                        detector_.getPose(i, tagSize, cam, cMo);
                        cMo_vec.push_back(cMo);
                    }
                }

                if (!cMo_vec.empty())
                {
                    pose_matrix = cMo_vec[0];
                    vpRotationMatrix R;
                    pose_matrix.extract(R);
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                        {
                            R_msg.data[i * 3 + j] = R[i][j];
                        }
                    }
                    vpTranslationVector t(0, -0.0584, 0);
                    t = R * t;
                    ros_pbvs_msg.data[0] = pose_matrix[0][3] - t[0];
                    ros_pbvs_msg.data[1] = pose_matrix[1][3] - t[1];
                    ros_pbvs_msg.data[2] = pose_matrix[2][3] - t[2];
                    ros_pbvs_msg.data[3] = 1;
                    ros_pbvs_msg.data[4] = ros::Time::now().toSec();
                }
                else
                {
                    R_msg.data = {1, 0, 0, 0, -1, 0, 0, 0, -1};
                    ros_pbvs_msg.data[3] = 0;
                    ros_pbvs_msg.data[4] = ros::Time::now().toSec();
                    std::cout << "lost!!!!!!" << std::endl;
                }
                auto delay = microseconds(40000) - duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_temp);
                if (delay.count() > 0)
                {
                    std::this_thread::sleep_for(delay);
                }
                auto delay2 = duration_cast<microseconds>(high_resolution_clock::now() - image_timestamp_temp);
                cout << delay2.count() << endl;
                pbvs_publisher.publish(ros_pbvs_msg);
                R_publisher.publish(R_msg);
                processing = false;
            }
            // 让线程稍作休息，避免空转
            std::this_thread::sleep_for(std::chrono::microseconds(100)); // 休眠 0.1 毫秒
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");
    argc_ = argc;
    argv_ = argv;
    cout << "argc: " << argc_ << endl;
    cout << "argv: " << argv_ << endl;
    ros::NodeHandle nh;
    ObjectDetector detector(nh);
    detector.start();
    return 0;
}
