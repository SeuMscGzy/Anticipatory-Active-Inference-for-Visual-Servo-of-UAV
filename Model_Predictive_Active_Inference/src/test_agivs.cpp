#include "test_agivs.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS()
    : nh("~"),
      u(0.0),
      optimizer(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
{
    relative_pos_sub = nh.subscribe("/x_state", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    u_pub = nh.advertise<std_msgs::Float64>("/input_x_axis", 1);
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    vector<double> optical_solution = {0, 0, 0};
    double mu = optical_solution[1];
    double mu_p = optical_solution[2];
    optical_solution = optimizer.optimize(msg->data[0], msg->data[1], mu, mu_p);
    u = optical_solution[0];
    std_msgs::Float64 u_msg;
    u_msg.data = u;
    u_pub.publish(u_msg);
}
