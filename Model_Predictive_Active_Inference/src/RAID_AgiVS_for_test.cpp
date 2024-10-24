#include "RAID_AgiVS_for_test.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS()
    : nh("~"),
      hat_x(Eigen::Vector2d::Zero()),
      predict_y(0.0),
      y_real(0.0),
      y_speed_real(0.0),
      u(0.0),
      timer_count(0),
      run_control_loop(false),
      optimizer(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
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
    relative_pos_sub = nh.subscribe("/x_state", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 10);
    u_pub = nh.advertise<std_msgs::Float64>("/input_x_axis", 1);
    timer = nh.createTimer(ros::Duration(0.01), &RAID_AgiVS::timerCallback, this);
}

void RAID_AgiVS::timerCallback(const ros::TimerEvent &)
{
    if (run_control_loop && timer_count < 4)
    {
        function();    // 执行控制逻辑
        timer_count++; // 增加计数
    }
    else
    {
        run_control_loop = false;
    }
}

void RAID_AgiVS::function()
{
    if (timer_count == 0)
    {
        predict_y = y_real;
        if (first_in_callback)
        {
            hat_x(0) = y_real;
            first_in_callback = false;
        }
        int mu = optical_solution[1];
        int mu_p = optical_solution[2];
        optical_solution = optimizer.optimize(hat_x(0), hat_x(1), mu, mu_p);
        u = optical_solution[0];
    }
    else
    {
        Eigen::Vector2d coeff(1, 0);
        predict_y = coeff.transpose() * (A0 * hat_x + B0 * u);
        int mu = optical_solution[1];
        int mu_p = optical_solution[2];
        optical_solution = optimizer.optimize(hat_x(0), hat_x(1), mu, mu_p);
        u = optical_solution[0];
    }
    std_msgs::Float64 u_msg;
    u_msg.data = u;
    u_pub.publish(u_msg);
    std_msgs::Float64MultiArray hat_x_msg;
    hat_x_msg.data.push_back(hat_x(0));
    hat_x_msg.data.push_back(hat_x(1));
    hat_x_msg.data.push_back(u);
    hat_x_msg.data.push_back(y_real);
    hat_x_msg.data.push_back(y_speed_real);
    hat_x_msg.data.push_back(optical_solution[1]);
    hat_x_msg.data.push_back(optical_solution[2]);
    hat_x_msg.data.push_back(optical_solution[3]);
    pub_hat_x.publish(hat_x_msg);
    // 更新状态
    hat_x = A_bar * hat_x + B_bar * u + C_bar * predict_y;
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    y_real = msg->data[0];
    y_speed_real = msg->data[1];
    timer_count = 0;
    run_control_loop = true;
    auto start = std::chrono::high_resolution_clock::now();
    function(); // 立即执行一次
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO_STREAM("Function execution time: " << elapsed.count() << " seconds");
}
