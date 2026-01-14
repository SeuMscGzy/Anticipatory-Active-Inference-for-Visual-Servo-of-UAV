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
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";

    odom.pose.pose = pose.pose;
    odom.twist.twist = vel.twist;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_pose_and_velocity");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_from_mocap", 1);

    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/GZY0/0/pose", 1, POSEcallback);
    ros::Subscriber twist_sub = nh.subscribe("/vrpn_client_node/GZY0/0/velocity", 1, VELcallback);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 160.0), timerCallback);

    ros::spin();
    return 0;
}
