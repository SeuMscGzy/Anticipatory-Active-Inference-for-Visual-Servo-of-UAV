#include "APO.h"

// 构造函数
APO::APO()
    : nh("~"),
      hat_tag_x(Eigen::Vector2d::Zero()),
      predict_tag_x(0.0),
      tag_x_real(0.0),
      hat_tag_y(Eigen::Vector2d::Zero()),
      predict_tag_y(0.0),
      tag_y_real(0.0),
      hat_tag_z(Eigen::Vector2d::Zero()),
      predict_tag_z(0.0),
      tag_z_real(0.0),
      uav_x(0.0),
      uav_y(0.0),
      uav_z(0.0),
      uav_vx(0.0),
      uav_vy(0.0),
      uav_vz(0.0),
      des_yaw(0.0),
      timer_count(0),
      loss_target(true),
      loss_or_not_(true),
      first_time_in_fun(true)
{
    A_bar << 0.458047258317164, 0.00704688089718714,
        -8.63242909905424, 0.951328921120263;
    C_bar << 0.541952741682836,
        8.63242909905424;
    A0 << 1, 0.0100000000000000,
        0, 1;
    relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &APO::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    Odom_sub = nh.subscribe("/mavros/local_position/odom", 1, &APO::Odom_Callback, this, ros::TransportHints().tcpNoDelay());
    timer = nh.createTimer(ros::Duration(0.02), &APO::timerCallback, this);
    hat_pub = nh.advertise<std_msgs::Float64MultiArray>("/hat_error_xyz", 1);
}

void APO::timerCallback(const ros::TimerEvent &)
{
    function(loss_or_not_); // 立即执行一次
    timer_count++;          // 增加计数
    if (is_data_refreshed)
    {
        timer_count = 0;
        is_data_refreshed = false;
    }
}

void APO::Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_x = msg->pose.pose.position.x;
    uav_y = msg->pose.pose.position.y;
    uav_z = msg->pose.pose.position.z;
    uav_vx = msg->twist.twist.linear.x;
    uav_vy = msg->twist.twist.linear.y;
    uav_vz = msg->twist.twist.linear.z;
}

void APO::function(bool loss_or_not_)
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
        is_data_refreshed = false;
        first_time_in_fun = false;
        predict_tag_x = tag_x_real;
        hat_tag_x(0) = tag_x_real;
        hat_tag_x(1) = 0;
        predict_tag_y = tag_y_real;
        hat_tag_y(0) = tag_y_real;
        hat_tag_y(1) = 0;
        predict_tag_z = tag_z_real;
        hat_tag_z(0) = tag_z_real;
        hat_tag_z(1) = 0;
    }
    else
    {
        if (timer_count == 0)
        {
            is_data_refreshed = false;
            predict_tag_x = tag_x_real;
            hat_tag_x = A_bar * hat_tag_x + C_bar * predict_tag_x;
            predict_tag_y = tag_y_real;
            hat_tag_y = A_bar * hat_tag_y + C_bar * predict_tag_y;
            predict_tag_z = tag_z_real;
            hat_tag_z = A_bar * hat_tag_z + C_bar * predict_tag_z;
        }
        else
        {
            Eigen::Vector2d coeff(1, 0);
            predict_tag_x = coeff.transpose() * A0 * hat_tag_x;
            hat_tag_x = A_bar * hat_tag_x + C_bar * predict_tag_x;
            predict_tag_y = coeff.transpose() * A0 * hat_tag_y;
            hat_tag_y = A_bar * hat_tag_y + C_bar * predict_tag_y;
            predict_tag_z = coeff.transpose() * A0 * hat_tag_z;
            hat_tag_z = A_bar * hat_tag_z + C_bar * predict_tag_z;
        }
    }
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(hat_tag_x(0) - uav_x);
    msg.data.push_back(hat_tag_x(1) - uav_vx);
    msg.data.push_back(hat_tag_y(0) - uav_y);
    msg.data.push_back(hat_tag_y(1) - uav_vy);
    msg.data.push_back(hat_tag_z(0) - uav_z);
    msg.data.push_back(hat_tag_z(1) - uav_vz);
    msg.data.push_back(tag_x_real - uav_x);
    msg.data.push_back(tag_y_real - uav_y);
    msg.data.push_back(tag_z_real - uav_z);
    msg.data.push_back(loss_or_not_);
    msg.data.push_back(des_yaw);
    hat_pub.publish(msg);
}

void APO::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    tag_x_real = msg->data[0] + uav_x;
    tag_y_real = msg->data[1] + uav_y;
    tag_z_real = msg->data[2] + uav_z;
    loss_or_not_ = msg->data[4];
    des_yaw = msg->data[5];
    is_data_refreshed = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "APO");
    APO apo;
    apo.spin();
    return 0;
}
