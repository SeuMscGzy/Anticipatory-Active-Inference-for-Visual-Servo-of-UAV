#include "RAID_AgiVS_for_UAV.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS()
    : nh("~"),
      px4_state(1),
      hat_x_last(Eigen::Vector2d::Zero()),
      hat_x(Eigen::Vector2d::Zero()),
      hat_y_last(Eigen::Vector2d::Zero()),
      hat_y(Eigen::Vector2d::Zero()),
      hat_z_last(Eigen::Vector2d::Zero()),
      hat_z(Eigen::Vector2d::Zero()),
      predict_x(0.0),
      x_real(0.0),
      predict_y(0.0),
      y_real(0.0),
      predict_z(0.0),
      z_real(0.0),
      timer_count(0),
      loss_target(true),
      loss_or_not_(true),
      use_bias_(true),
      optimizer_x(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, e1, precice_z_u, umin, umax),
      optimizer_y(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, e1, precice_z_u, umin, umax),
      optimizer_z(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, e1, precice_z_u, umin, umax),
      first_time_in_fun(true)
{
    A_bar << 0.518572754477203, 0.00740818220681718,
        -6.66736398613546, 0.963063686886233;
    B_bar << 0,
        0;
    C_bar << 0.481427245526137,
        6.66736398622287;
    A0 << 1, 0.0100000000000000,
        0, 1;
    B0 << 5.00000000000000e-05,
        0.0100000000000000;
    px4_state_sub = nh.subscribe("/px4_state_pub", 1, &RAID_AgiVS::StateCallback, this, ros::TransportHints().tcpNoDelay());
    relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &RAID_AgiVS::relative_pos_Callback, this);
    pub_land = nh.advertise<std_msgs::Bool>("/flight_land", 1);
}

double RAID_AgiVS::adjustBias(double value, int which_axis, bool use_bias) // which_axis: 0 for x, 1 for y, 2 for z
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

void RAID_AgiVS::cal_ctrl_input(double loss_or_not, bool use_bias)
{
    loss_or_not_ = loss_or_not;
    x_real = adjustBias(x_real, 0, use_bias);
    y_real = adjustBias(y_real, 1, use_bias);
    z_real = adjustBias(z_real, 2, use_bias);
    function(loss_or_not_);
    timer_count++;
    timer = nh.createTimer(ros::Duration(0.01), &RAID_AgiVS::timerCallback, this);
}

void RAID_AgiVS::timerCallback(const ros::TimerEvent &)
{
    function(loss_or_not_);
    timer_count++;
    cout << "timer_count: " << timer_count << endl;
    if (timer_count >= 5)
    {
        timer.stop();
        timer_count = 0;
    }
}

void RAID_AgiVS::function(bool loss_or_not)
{
    if (loss_or_not && loss_target == false) // 从能看到目标到看不到目标
    {
        loss_target = true;
    }
    if (!loss_or_not && loss_target == true) // 从不能看到目标到能看到目标
    {
        loss_target = false;
        first_time_in_fun = true;
    }
    if (first_time_in_fun)
    {
        first_time_in_fun = false;
        predict_x = x_real;
        predict_y = y_real;
        predict_z = z_real;
        hat_x(0) = x_real;
        hat_x(1) = 0;
        hat_y(0) = y_real;
        hat_y(1) = 0;
        hat_z(0) = z_real;
        hat_z(1) = 0;
        cal_optical_ctrl();
        u_x = optical_x[0];
        u_y = optical_y[0];
        u_z = optical_z[0];
    }
    else
    {
        if (timer_count == 0)
        {
            predict_x = x_real;
            predict_y = y_real;
            predict_z = z_real;
            u_x = 0;
            u_y = 0;
            u_z = 0;
            hat_x = A_bar * hat_x_last + B_bar * u_x + C_bar * predict_x;
            hat_y = A_bar * hat_y_last + B_bar * u_y + C_bar * predict_y;
            hat_z = A_bar * hat_z_last + B_bar * u_z + C_bar * predict_z;
            cal_optical_ctrl();
            u_x = optical_x[0];
            u_y = optical_y[0];
            u_z = optical_z[0];
        }
        else
        {
            Eigen::Vector2d coeff(1, 0);
            u_x = 0;
            u_y = 0;
            u_z = 0;
            predict_x = coeff.transpose() * (A0 * hat_x_last + B0 * u_x);
            hat_x = A_bar * hat_x_last + B_bar * u_x + C_bar * predict_x;
            predict_y = coeff.transpose() * (A0 * hat_y_last + B0 * u_y);
            hat_y = A_bar * hat_y_last + B_bar * u_y + C_bar * predict_y;
            predict_z = coeff.transpose() * (A0 * hat_z_last + B0 * u_z);
            hat_z = A_bar * hat_z_last + B_bar * u_z + C_bar * predict_z;
            cal_optical_ctrl();
            u_x = optical_x[0];
            u_y = optical_y[0];
            u_z = optical_z[0];
        }
    }
    // Update last values for the next iteration
    hat_x_last = hat_x;
    hat_y_last = hat_y;
    hat_z_last = hat_z;
}

void RAID_AgiVS::cal_optical_ctrl()
{
    double mux = optical_x[1];
    double mux_p = optical_x[2];
    optical_x = optimizer_x.optimize(hat_x(0), hat_x(1), mux, mux_p);
    double muy = optical_y[1];
    double muy_p = optical_y[2];
    optical_y = optimizer_y.optimize(hat_y(0), hat_y(1), muy, muy_p);
    double muz = optical_z[1];
    double muz_p = optical_z[2];
    optical_z = optimizer_z.optimize(hat_z(0), hat_z(1), muz, muz_p);
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    relative_pos.data = msg->data;
    x_real = relative_pos.data[0];
    y_real = relative_pos.data[1];
    z_real = relative_pos.data[2];
    if (land_or_just_tracking)
    {
        if (msg->data[0] > 0.15 || msg->data[1] > 0.15)
        {
            cal_ctrl_input(relative_pos.data[4], 1);
            std_msgs::Bool land_msg;
            land_msg.data = false;
            pub_land.publish(land_msg);
        }
        else
        {
            cal_ctrl_input(relative_pos.data[4], 0);
            if ((msg->data[2] < 0.15 && msg->data[4] == 0 && px4_state == 3) || keep_in_land)
            {
                // 发送强制着陆指令，因为可能快要看不到目标了，开环近距离着陆
                keep_in_land = true;
                std_msgs::Bool land_msg;
                land_msg.data = true;
                pub_land.publish(land_msg);
                cout << "land message sent" << endl;
            }
        }
    }
    else
    {
        cal_ctrl_input(relative_pos.data[4], 1);
        std_msgs::Bool land_msg;
        land_msg.data = false;
        pub_land.publish(land_msg);
    }
}

void RAID_AgiVS::StateCallback(const std_msgs::Int32::ConstPtr &msg)
{
    px4_state = msg->data;
    if (msg->data != 3) // 不在cmd模式下时，控制量为0；
    {
        u_x = 0;
        u_y = 0;
        u_z = 0;
    }
}
