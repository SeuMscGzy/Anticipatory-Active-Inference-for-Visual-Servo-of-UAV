#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
using namespace std;
using namespace Eigen;

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                       q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
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
    Eigen::Vector3d Position_before = Eigen::Vector3d::Zero();
    Eigen::Vector3d Position_after = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_c2a = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rot_matrix3; // Ric imu到相机
    double desired_yaw;

public:
    AprilTagDetector()
    {
        desired_yaw = 0;
        // 初始化 rot_matrix3
        rot_matrix3 << -0.012299254251302336, -0.9997550667398611, -0.018399317184020714,
            -0.9988718281685636, 0.011440175054767979, 0.046088971413001924,
            -0.04586719128150388, 0.018945419570245543, -0.998767876856907;
        R = rot_matrix3;
        point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1);
        image_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(
                        "/object_pose", 1,
                        &AprilTagDetector::imageCb, this,
                        ros::TransportHints().tcpNoDelay());

        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/vins_fusion/imu_propagate", 1,
            &AprilTagDetector::odomCallback, this,
            ros::TransportHints().tcpNoDelay());

        R_subscriber = nh_.subscribe<std_msgs::Float64MultiArray>(
            "/R_data", 1,
            &AprilTagDetector::RCallback, this,
            ros::TransportHints().tcpNoDelay());
    }

    ~AprilTagDetector()
    {
    }

    void publishDetectionResult(bool lost)
    {
        point_.data = {Position_after(0), Position_after(1), Position_after(2),
                       image_time, static_cast<double>(lost), desired_yaw};
        point_pub_.publish(point_);
        point_.data.clear();
    }

    void RCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() == 9)
        {
            // 使用 Eigen::Map 直接将数据映射到矩阵
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> map_matrix(msg->data.data());
            R_c2a = map_matrix;
        }
        else
        {
            ROS_ERROR("Received matrix data does not have 9 elements.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        Eigen::Quaterniond q1(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        Eigen::Matrix3d rot_matrix1 = q1.toRotationMatrix(); // R_W2i  世界系到IMU
        R = rot_matrix1 * rot_matrix3; //世界系到相机系
    }

    void imageCb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        image_time = msg->data[4];
        if (msg->data[3] == 1)
        {
            Position_before(0) = msg->data[0] + 0.001125032938074339;
            Position_before(1) = msg->data[1] - 0.04185436595006169;
            Position_before(2) = msg->data[2] + 0.03678888970637513; // 转换到IMU所在位置
            Position_after = R * Position_before;
            Eigen::Matrix3d R_W2a = R * R_c2a;
            Eigen::Quaterniond q(R_W2a);
            double yaw = fromQuaternion2yaw(q);
            yaw = yaw + M_PI / 2;
            cout << yaw << endl;
            desired_yaw = yaw;
            publishDetectionResult(false);
        }
        else
        {
            desired_yaw = 0;
            publishDetectionResult(true);
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