#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "fake_pose_publisher");
    ros::NodeHandle nh;

    // 发布话题，与 VRPN 类似，例如：/TrackerName/pose
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/MCServer/5/pose", 10);

    ros::Rate rate(10); // 发布频率为 10Hz
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double elapsed = (current_time - start_time).toSec();

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "world"; // 可根据需要修改坐标系名称

        // 定义位移随时间变动：x 随时间线性增长
        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 1;

        // 使用 tf2 将 RPY 转换为四元数
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // 发布消息
        pose_pub.publish(pose_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
