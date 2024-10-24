#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
using namespace std;
double data2[] = {-0.012299254251302336, -0.9997550667398611, -0.018399317184020714,
                  -0.9988718281685636, 0.011440175054767979, 0.046088971413001924, -0.04586719128150388, 0.018945419570245543, -0.998767876856907};
tf::Matrix3x3 convertCvMatToTfMatrix(const cv::Mat &cv_matrix)
{
    tf::Matrix3x3 tf_matrix;
    if (cv_matrix.rows == 3 && cv_matrix.cols == 3 && cv_matrix.type() == CV_64F)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                tf_matrix[i][j] = cv_matrix.at<double>(i, j);
            }
        }
    }
    else
    {
        std::cerr << "Invalid matrix size or type" << std::endl;
    }
    return tf_matrix;
} 
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}
class AprilTagDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    std_msgs::Float64MultiArray point_;
    ros::Subscriber odom_sub_;
    ros::Publisher point_pub_;
    ros::Subscriber R_subscriber;
    double image_time;
    cv::Mat Position_before = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Position_after = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    tf::Matrix3x3 R_c2a;
    double desired_yaw;

public:
    AprilTagDetector()
    {
        desired_yaw = 0;
        cv::Mat tempMat(3, 3, CV_64F);
        tempMat = cv::Mat(3, 3, CV_64F, data2);
        R = tempMat;
        image_sub_ = nh_.subscribe("/object_pose", 1, &AprilTagDetector::imageCb, this);
        odom_sub_ = nh_.subscribe("/vins_fusion/imu_propagate", 1, &AprilTagDetector::odomCallback, this);
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1);
        R_subscriber = nh_.subscribe("/R_data", 1, &AprilTagDetector::RCallback, this);
    }

    ~AprilTagDetector()
    {
    }

    void publishDetectionResult(bool lost)
    {
        // 简化为直接赋值
        point_.data = {Position_after.at<double>(0, 0), Position_after.at<double>(1, 0), Position_after.at<double>(2, 0), image_time, static_cast<double>(lost), desired_yaw};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void RCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() == 9)
        {
            // 更新类中的旋转矩阵
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R_c2a[i][j] = msg->data[i * 3 + j];
                }
            }
        }
        else
        {
            ROS_ERROR("Received matrix data does not have 9 elements.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Quaternion q1(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 rot_matrix1(q1); // R_W2i  世界系到imu
        tf::Matrix3x3 rot_matrix3(-0.012299254251302336, -0.9997550667398611, -0.018399317184020714,
                  -0.9988718281685636, 0.011440175054767979, 0.046088971413001924, -0.04586719128150388, 0.018945419570245543, -0.998767876856907); // Ric imu到相机
        tf::Matrix3x3 rot_matrix_cam_to_world;
        rot_matrix_cam_to_world = rot_matrix1 * rot_matrix3;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.at<double>(i, j) = rot_matrix_cam_to_world[i][j];
            }
        }
    }
    void imageCb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        std_msgs::Float64MultiArray temp_array;
        temp_array.data = msg->data;
        image_time = temp_array.data[4];
        // cout << "pos_recieve-img_recieve: " << 1000 * (ros::Time::now().toSec() - image_time) << " ms" << endl;
        if (temp_array.data[3] == 1)
        {
            Position_before.at<double>(0, 0) = temp_array.data[0] + 0.001125032938074339;
            Position_before.at<double>(1, 0) = temp_array.data[1] - 0.04185436595006169;
            Position_before.at<double>(2, 0) = temp_array.data[2] + 0.03678888970637513; // 将其转换到imu飞控所在位置
            tf::Matrix3x3 R_tf = convertCvMatToTfMatrix(R);
            tf::Matrix3x3 R_W2a = R_tf * R_c2a;
            Eigen::Matrix3d eigen_mat;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    eigen_mat(i, j) = R_W2a[i][j];
                }
            }
            Eigen::Quaterniond q;
            q = Eigen::Quaterniond(eigen_mat);
            double yaw = fromQuaternion2yaw(q);
            yaw = yaw + M_PI / 2;
            cout << yaw << endl;
            desired_yaw = yaw;
            // cout << "Position: (" << Position_before.at<double>(0, 0) << ", " << Position_before.at<double>(1, 0) << ", " << Position_before.at<double>(2, 0) << ")" << endl;
            Position_after = R * Position_before;
            // cout << "Position: (" << Position_after.at<double>(0, 0) << ", " << Position_after.at<double>(1, 0) << ", " << Position_after.at<double>(2, 0) << ")" << endl;
            publishDetectionResult(false); // 提取的函数，用于发布检测结果
        }
        else
        {
            desired_yaw = 0;
            publishDetectionResult(true); // 提取的函数，用于发布检测结果
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_detector");
    AprilTagDetector atd;
    ros::spin();
    return 0;
}
