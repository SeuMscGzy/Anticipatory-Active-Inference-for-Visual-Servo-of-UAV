#include "RAID_AgiVS_for_UAV.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS(int index)
    : nh("~"),
      which_axis(index),
      px4_state(1),
      hat_x_last(Eigen::Vector2d::Zero()),
      hat_x(Eigen::Vector2d::Zero()),
      predict_x(0.0),
      x_real(0.0),
      timer_count(0),
      loss_target(true),
      loss_or_not_(true),
      optimizer_x(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, e1, precice_z_u, umin, umax),
      first_time_in_fun(true)
{
    A_bar << 0.458047258317164, 0.00704688089718714,
        -8.63242909905424, 0.951328921120263;
    B_bar << 0,
        0;
    C_bar << 0.541952741682836,
        8.63242909905424;
    A0 << 1, 0.0100000000000000,
        0, 1;
    B0 << 5.00000000000000e-05,
        0.0100000000000000;
    px4_state_sub = nh.subscribe("/px4_state_pub", 1, &RAID_AgiVS::StateCallback, this, ros::TransportHints().tcpNoDelay());
    relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    timer = nh.createTimer(ros::Duration(0.01), &RAID_AgiVS::timerCallback, this);
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

void RAID_AgiVS::timerCallback(const ros::TimerEvent &)
{
    if (run_control_loop && timer_count < 4)
    {
        function(loss_or_not_); // 立即执行一次
        timer_count++;          // 增加计数
    }
    else
    {
        run_control_loop = false;
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

void RAID_AgiVS::function(bool loss_or_not_)
{
    if (loss_or_not_ && loss_target == false) // 从能看到目标到看不到目标
    {
        loss_target = true;
    }
    if (!loss_or_not_ && loss_target == true) // 从不能看到目标到能看到目标
    {
        loss_target = false;
        first_time_in_fun = true;
    }
    if (first_time_in_fun)
    {
        first_time_in_fun = false;
        predict_x = x_real;
        hat_x(0) = x_real;
        hat_x(1) = 0;
        cal_optical_ctrl();
        u_x = optical_x[0];
    }
    else
    {
        if (timer_count == 0)
        {
            predict_x = x_real;
            u_x = 0;
            hat_x = A_bar * hat_x_last + B_bar * u_x + C_bar * predict_x;
            cal_optical_ctrl();
            u_x = optical_x[0];
        }
        else
        {
            Eigen::Vector2d coeff(1, 0);
            u_x = 0;
            predict_x = coeff.transpose() * (A0 * hat_x_last + B0 * u_x);
            hat_x = A_bar * hat_x_last + B_bar * u_x + C_bar * predict_x;
            cal_optical_ctrl();
            u_x = optical_x[0];
        }
    }
    // Update last values for the next iteration
    hat_x_last = hat_x;
    std_msgs::Float64 u_msg;
    u_msg.data = u_x;
    pub_u.publish(u_msg);
}

void RAID_AgiVS::cal_optical_ctrl()
{
    double mux = optical_x[1];
    double mux_p = optical_x[2];
    optical_x = optimizer_x.optimize(hat_x(0), hat_x(1), mux, mux_p);
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    relative_pos.data = msg->data;
    x_real = relative_pos.data[which_axis];
    if (px4_state != 3) // 不在cmd模式下时，控制量为0；
    {
        u_x = 0;
        run_control_loop = false;
        std_msgs::Float64 u_msg;
        u_msg.data = u_x;
        pub_u.publish(u_msg);
    }
    else
    {
        if (land_or_just_tracking)
        {
            if (msg->data[0] > 0.15 || msg->data[1] > 0.15)
            {
                loss_or_not_ = relative_pos.data[4];
                x_real = adjustBias(x_real, 1);
                timer_count = 0;
                run_control_loop = true;
                function(loss_or_not_);
            }
            else
            {
                loss_or_not_ = relative_pos.data[4];
                x_real = adjustBias(x_real, 0);
                timer_count = 0;
                run_control_loop = true;
                function(loss_or_not_);
            }
        }
        else
        {
            loss_or_not_ = relative_pos.data[4];
            x_real = adjustBias(x_real, 1);
            timer_count = 0;
            run_control_loop = true;
            function(loss_or_not_);
        }
    }
}

void RAID_AgiVS::StateCallback(const std_msgs::Int32::ConstPtr &msg)
{
    px4_state = msg->data;
}
