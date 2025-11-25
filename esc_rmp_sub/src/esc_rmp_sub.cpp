#include <ros/ros.h>
#include <mavros_msgs/ESCStatus.h>

void escStatusCb(const mavros_msgs::ESCStatus::ConstPtr& msg)
{
  for (size_t i = 0; i < msg->esc_status.size(); ++i)
  {
    int32_t rpm = msg->esc_status[i].rpm;   // 电机 i 的转速 (RPM)
    float voltage = msg->esc_status[i].voltage;
    float current = msg->esc_status[i].current;

    ROS_INFO("ESC %zu: rpm=%d, U=%.2f V, I=%.2f A",
             i, rpm, voltage, current);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esc_rpm_reader");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/mavros/esc_status", 10, escStatusCb);

  ros::spin();
  return 0;
}