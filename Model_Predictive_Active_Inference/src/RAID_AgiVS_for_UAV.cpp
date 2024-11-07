#include "RAID_AgiVS_for_UAV.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS(int index)
    : nh("~"),
      which_axis(index),
      px4_state(1),
      x_real(0.0),
      xv_real(0.0),
      optimizer_x(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, e1, precice_z_u, umin, umax)
{
    px4_state_sub = nh.subscribe("/px4_state_pub", 1, &RAID_AgiVS::StateCallback, this, ros::TransportHints().tcpNoDelay());
    relative_pos_sub = nh.subscribe("/hat_error_xyz", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    if (which_axis == 0)
    {
        pub_u = nh.advertise<std_msgs::Float64>("/input_x_axis", 1);
    }
    else if (which_axis == 1)
    {
        pub_u = nh.advertise<std_msgs::Float64>("/input_y_axis", 1);
    }
    else
    {
        pub_u = nh.advertise<std_msgs::Float64>("/input_z_axis", 1);
    }
}

double RAID_AgiVS::adjustBias(double value, bool use_bias) // which_axis: 0 for x, 1 for y, 2 for z
{
    if (use_bias)
    {
        if (which_axis == 0)
        {
            return value + 0;
        }
        else if (which_axis == 1)
        {
            return value + 0;
        }
        else
        {
            return value + 1;
        }
    }
    else
    {
        return value;
    }
}

void RAID_AgiVS::cal_optical_ctrl()
{
    double mux = optical_x[1];
    double mux_p = optical_x[2];
    optical_x = optimizer_x.optimize(x_real, xv_real, mux, mux_p);
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    relative_pos.data = msg->data;
    x_real = relative_pos.data[2 * which_axis];
    xv_real = relative_pos.data[2 * which_axis + 1];
    if (px4_state != 3) // 不在cmd模式下时，控制量为0；
    {
        u_x = 0;
        std_msgs::Float64 u_msg;
        u_msg.data = u_x;
        pub_u.publish(u_msg);
    }
    else
    {
        if (land_or_just_tracking)
        {
            if (abs(msg->data[0] > 0.15) || abs(msg->data[2] > 0.15))
            {
                x_real = adjustBias(x_real, 1);
            }
            else
            {
                x_real = adjustBias(x_real, 0);
            }
            cal_optical_ctrl();
            u_x = optical_x[0];
            std_msgs::Float64 u_msg;
            u_msg.data = u_x;
            pub_u.publish(u_msg);
        }
        else
        {
            x_real = adjustBias(x_real, 1);
            cal_optical_ctrl();
            u_x = optical_x[0];
            std_msgs::Float64 u_msg;
            u_msg.data = u_x;
            pub_u.publish(u_msg);
        }
    }
}

void RAID_AgiVS::StateCallback(const std_msgs::Int32::ConstPtr &msg)
{
    px4_state = msg->data;
}
