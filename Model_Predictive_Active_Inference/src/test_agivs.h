#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "mpai_qp_ty.h"
#include <std_msgs/Int32.h>
using namespace std;
using namespace Eigen;
class RAID_AgiVS
{
public:
    double dt = 0.01;
    int Np = 100;
    double precice_z1 = 4;
    double precice_z2 = 0.5;
    double precice_w1 = 2;
    double precice_w2 = 1;
    double precice_z_u = 0.005;
    double e1 = 3;
    double umin = -4;
    double umax = 4;
    Optimizer optimizer;
    ros::NodeHandle nh;
    // Assuming we have these initial values from some measurements
    ros::Publisher u_pub;
    double u;
    ros::Subscriber relative_pos_sub;
    ros::Subscriber relative_speed_sub;
    std_msgs::Float64 relative_pos;
    std_msgs::Float64 relative_speed;
    // 构造函数
    RAID_AgiVS();
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};