// src/quat_array_to_csv_subscriber.cpp

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <iomanip>

static const std::string CSV_PATH = "quat_array.csv";

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 8) {
    ROS_WARN("quat_array length %zu < 8", msg->data.size());
    return;
  }

  // 打开文件（追加模式）
  std::ofstream fout(CSV_PATH, std::ios::app);
  if (!fout.is_open()) {
    ROS_ERROR("Failed to open %s", CSV_PATH.c_str());
    return;
  }

  fout << std::fixed << std::setprecision(6)
       // u_q
       << msg->data[0] << ',' 
       << msg->data[1] << ',' 
       << msg->data[2] << ',' 
       << msg->data[3] << ','
       // q_se3
       << msg->data[4] << ',' 
       << msg->data[5] << ',' 
       << msg->data[6] << ',' 
       << msg->data[7]
       << '\n';
  fout.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quat_array_to_csv_subscriber");
  ros::NodeHandle nh;

  // 启动时写入表头（覆盖模式）
  {
    std::ofstream fout(CSV_PATH, std::ios::out);
    if (!fout.is_open()) {
      ROS_ERROR("Failed to create %s", CSV_PATH.c_str());
      return 1;
    }
    fout << "uq_w,uq_x,uq_y,uq_z,s3_w,s3_x,s3_y,s3_z\n";
    fout.close();
    ROS_INFO("Created CSV file and wrote header: %s", CSV_PATH.c_str());
  }

  ros::Subscriber sub =
    nh.subscribe("quat_array", 10, callback);

  ros::spin();
  return 0;
}
