#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped vel;

void POSEcallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    pose = *pose_msg;
}

void VELcallback(const geometry_msgs::TwistStamped::ConstPtr &twist_msg)
{
    vel = *twist_msg;
}

void timerCallback(const ros::TimerEvent &)
{
    nav_msgs::Odometry odom;
    // 填充消息
    odom.header.stamp = ros::Time::now(); // 或者使用 pose.header.stamp 或 vel.header.stamp
    odom.header.frame_id = "world";

    // 需要从 PoseStamped 和 TwistStamped 中提取 Pose 和 Twist
    odom.pose.pose = pose.pose;
    odom.twist.twist = vel.twist;

    // 发布消息
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_pose_and_velocity");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 1);

    // 订阅位姿和速度消息
    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/MCServer/5/pose", 1, POSEcallback);
    ros::Subscriber twist_sub = nh.subscribe("/vrpn_client_node/MCServer/5/velocity", 1, VELcallback);

    // 创建定时器以160Hz的频率调用timerCallback
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 160.0), timerCallback);

    ros::spin();
    return 0;
}
