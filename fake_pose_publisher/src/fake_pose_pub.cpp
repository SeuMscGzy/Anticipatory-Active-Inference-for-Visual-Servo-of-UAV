#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_mocap_pose_pub");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 可以通过私有参数改话题名：_topic:=...
  std::string topic_name;
  pnh.param<std::string>("topic", topic_name,
                         std::string("/mavros/vision_pose/pose"));

  ros::Publisher pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);

  double rate_hz;
  pnh.param("rate", rate_hz, 100.0);   // 发布频率
  ros::Rate rate(rate_hz);

  // 虚构一个固定的位姿
  const double x = 1;
  const double y = 0;
  const double z = 0;
  const double roll  = 0.0;
  const double pitch = 0.0;
  const double yaw   = 0; // rad

  ROS_INFO_STREAM("Fake mocap pose publisher on [" << topic_name
                  << "], rate = " << rate_hz << " Hz");

  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";   // 你期望的世界坐标系名称

    // 位置
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    // 姿态（RPY -> 四元数）
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pose_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
