#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "fake_pose_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 10);
    ros::Rate rate(1.0 / 0.06); // 1/0.06 = 16.67Hz

    std_msgs::Float64MultiArray msg;
    msg.data = {1, 1, 1, 0, 0, 0}; // 设定固定数据
    double a = ros::Time::now().toSec();
    while (ros::ok())
    {
        double b = sin(ros::Time::now().toSec() - a - 0.058);
        double c = sin(ros::Time::now().toSec() - a);
        msg.data[0] = b;
        msg.data[5] = c;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
