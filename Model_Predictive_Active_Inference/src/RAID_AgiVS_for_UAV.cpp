#include "RAID_AgiVS_for_UAV.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS()
    : nh("~"),
      spinner(5),
      px4_state(1),
      x_real(0.0),
      xv_real(0.0),
      y_real(0.0),
      yv_real(0.0),
      z_real(0.0),
      zv_real(0.0),
      u_x(0.0),
      u_y(0.0),
      u_z(0.0),
      des_yaw(0.0),
      optimizer_x(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax),
      optimizer_y(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax),
      optimizer_z(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
{
    acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
    pub_land = nh.advertise<std_msgs::Bool>("/flight_land", 1);

    px4_state_sub = nh.subscribe("/px4_state_pub", 1, &RAID_AgiVS::StateCallback, this, ros::TransportHints().tcpNoDelay());
    relative_pos_sub = nh.subscribe("/hat_error_xyz", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    // 启动 ROS 异步 spinner
    spinner.start();

    // 启动控制循环线程
    startControlLoops();
}

RAID_AgiVS::~RAID_AgiVS()
{
    // 停止 spinner
    spinner.stop();

    // 确保控制线程安全退出
    if (x_thread.joinable())
        x_thread.join();
    if (y_thread.joinable())
        y_thread.join();
    if (z_thread.joinable())
        z_thread.join();
    if (u_pub_thread.joinable())
        u_pub_thread.join();
}

void RAID_AgiVS::startControlLoops()
{
    x_thread = std::thread(&RAID_AgiVS::xAxisControlLoop, this);
    y_thread = std::thread(&RAID_AgiVS::yAxisControlLoop, this);
    z_thread = std::thread(&RAID_AgiVS::zAxisControlLoop, this);
    u_pub_thread = std::thread(&RAID_AgiVS::uPubLoop, this);
}

void RAID_AgiVS::xAxisControlLoop()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        double mux = optical_x[1];
        double mux_p = optical_x[2];
        double x = x_real;
        double xv = xv_real;
        optical_x = optimizer_x.optimize(x, xv, mux, mux_p);
        u_x = optical_x[0];
        rate.sleep();
    }
}

void RAID_AgiVS::yAxisControlLoop()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        double muy = optical_y[1];
        double muy_p = optical_y[2];
        double y = y_real;
        double yv = yv_real;
        optical_y = optimizer_y.optimize(y, yv, muy, muy_p);
        u_y = optical_y[0];
        rate.sleep();
    }
}

void RAID_AgiVS::zAxisControlLoop()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        double muz = optical_z[1];
        double muz_p = optical_z[2];
        double z = z_real;
        double zv = zv_real;
        optical_z = optimizer_z.optimize(z, zv, muz, muz_p);
        u_z = optical_z[0];
        rate.sleep();
    }
}

void RAID_AgiVS::uPubLoop()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        acc_msg.position.x = 0;
        acc_msg.position.y = 0;
        acc_msg.position.z = 0;
        acc_msg.velocity.x = 0;
        acc_msg.velocity.y = 0;
        acc_msg.velocity.z = 0;
        acc_msg.acceleration.x = u_x;
        acc_msg.acceleration.y = u_y;
        acc_msg.acceleration.z = u_z;
        acc_msg.jerk.x = 0;
        acc_msg.jerk.y = 0;
        acc_msg.jerk.z = 0;
        acc_msg.yaw = des_yaw;
        acc_msg.yaw_dot = 0;
        acc_msg.header.frame_id = "world";
        acc_cmd_pub.publish(acc_msg);
        rate.sleep();
    }
}

void RAID_AgiVS::adjustBias(bool use_bias) // which_axis: 0 for x, 1 for y, 2 for z
{
    if (land_or_just_tracking)
    {
        if (use_bias)
        {
            z_real = z_real + 1;
        }
    }
    else // just tracking
    {
        if (use_bias)
        {
            x_real = x_real + 0.1;
            y_real = y_real + 0.1;
            z_real = z_real + 1;
        }
    }
}

void RAID_AgiVS::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    x_real = msg->data[0];
    xv_real = msg->data[1];
    y_real = msg->data[2];
    yv_real = msg->data[3];
    z_real = msg->data[4];
    zv_real = msg->data[5];
    des_yaw = msg->data[10];
    if (land_or_just_tracking)
    {
        if (abs(msg->data[6] > 0.15) || abs(msg->data[7] > 0.15))
        {
            adjustBias(true);
            std_msgs::Bool land_msg;
            land_msg.data = false;
            pub_land.publish(land_msg);
        }
        else
        {
            if ((msg->data[8] < 0.15 && msg->data[9] == 0 && px4_state == 3) || keep_in_land)
            {
                // Send forced landing command as we might be losing sight of the target, open-loop close proximity landing
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
        adjustBias(true);
        std_msgs::Bool land_msg;
        land_msg.data = false;
        pub_land.publish(land_msg);
    }
}

void RAID_AgiVS::StateCallback(const std_msgs::Int32::ConstPtr &msg)
{
    px4_state = msg->data;
}
