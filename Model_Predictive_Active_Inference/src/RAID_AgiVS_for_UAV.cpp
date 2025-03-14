#include "RAID_AgiVS_for_UAV.h"

// 构造函数
RAID_AgiVS::RAID_AgiVS(ros::NodeHandle &nh)
    : nh(nh),
      px4_state(3),
      x_real(0.0),
      xv_real(0.0),
      y_real(0.0),
      yv_real(0.0),
      z_real(0.0),
      zv_real(0.0),
      des_yaw(0.0),
      optimizer_xyz(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
{
    acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
    pub_land = nh.advertise<std_msgs::Bool>("/flight_land", 1);
    x_pub = nh.advertise<std_msgs::Float64>("/input_x_axis", 1);
    y_pub = nh.advertise<std_msgs::Float64>("/input_y_axis", 1);
    z_pub = nh.advertise<std_msgs::Float64>("/input_z_axis", 1);
    px4_state_sub = nh.subscribe("/px4_state_pub", 1, &RAID_AgiVS::StateCallback, this, ros::TransportHints().tcpNoDelay());
    relative_pos_sub = nh.subscribe("/hat_error_xyz", 1, &RAID_AgiVS::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());

    // 启动控制循环线程
    startControlLoops();
}

RAID_AgiVS::~RAID_AgiVS()
{
    // 确保控制线程安全退出
    if (xyz_thread.joinable())
        xyz_thread.join();
}

void RAID_AgiVS::startControlLoops()
{
    xyz_thread = std::thread(&RAID_AgiVS::xyzAxisControlLoop, this);
}

void RAID_AgiVS::xyzAxisControlLoop()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        if (px4_state == 3)
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            Vector3d xyz_real(x_real, y_real, z_real);
            Vector3d xyz_v_real(xv_real, yv_real, zv_real);
            Vector3d optical_mu(optical_xyz[3], optical_xyz[4], optical_xyz[5]);
            Vector3d optical_mu_p(optical_xyz[6], optical_xyz[7], optical_xyz[8]);
            optical_xyz = optimizer_xyz.optimize(xyz_real, xyz_v_real, optical_mu, optical_mu_p);
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
            ROS_INFO("Execution time x: %f seconds", elapsed_time);
            //cout<<"optical_xyz: "<<optical_xyz[0]<<" "<<optical_xyz[1]<<" "<<optical_xyz[2]<<endl;
            cout<< xyz_real << endl;
            cout<< xyz_v_real << endl;
            acc_msg.position.x = 0;
            acc_msg.position.y = 0;
            acc_msg.position.z = 0;
            acc_msg.velocity.x = 0;
            acc_msg.velocity.y = 0;
            acc_msg.velocity.z = 0;
            acc_msg.acceleration.x = optical_xyz[0];
            acc_msg.acceleration.y = optical_xyz[1];
            acc_msg.acceleration.z = optical_xyz[2];
            acc_msg.jerk.x = 0;
            acc_msg.jerk.y = 0;
            acc_msg.jerk.z = 0;
            acc_msg.yaw = des_yaw;
            acc_msg.yaw_dot = 0;
            acc_msg.header.frame_id = "world";
            acc_cmd_pub.publish(acc_msg);
            std_msgs::Float64 x_msg;
            x_msg.data = optical_xyz[0];
            x_pub.publish(x_msg);
            std_msgs::Float64 y_msg;
            y_msg.data = optical_xyz[1];
            y_pub.publish(y_msg);
            std_msgs::Float64 z_msg;
            z_msg.data = optical_xyz[2];
            z_pub.publish(z_msg);
            rate.sleep();
        }
        else
        {
            acc_msg.position.x = 0;
            acc_msg.position.y = 0;
            acc_msg.position.z = 0;
            acc_msg.velocity.x = 0;
            acc_msg.velocity.y = 0;
            acc_msg.velocity.z = 0;
            acc_msg.acceleration.x = 0;
            acc_msg.acceleration.y = 0;
            acc_msg.acceleration.z = 0;
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
            // x_real = x_real + 0.1;
            // y_real = y_real + 0.1;
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
            if ((abs(msg->data[8]) < 0.15 && msg->data[9] == 0 && px4_state == 3) || keep_in_land)
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
