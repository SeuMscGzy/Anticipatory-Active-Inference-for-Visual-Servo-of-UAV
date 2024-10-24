#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_send_thrust");
    ros::NodeHandle nh;

    // Publisher for fake odometry information
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10);

    // Publisher for thrust value
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    ros::Rate loop_rate(10);  // Adjust the loop rate as per your requirements

    while (ros::ok())
    {
        // Create and publish fake odometry message
        nav_msgs::Odometry odom_msg;
        // Fill in the necessary fields of the odom_msg
        // ...
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        odom_pub.publish(odom_msg);

        // Create and publish thrust message
        mavros_msgs::AttitudeTarget thrust_msg;
        thrust_msg.thrust = 0.1;
        //thrust_pub.publish(thrust_msg);
        // Keyboard input
        /*if (std::cin.peek() != EOF) {
            int input;
            std::cin >> input;
            if (input == 1) {
           
            }
        }*/
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}