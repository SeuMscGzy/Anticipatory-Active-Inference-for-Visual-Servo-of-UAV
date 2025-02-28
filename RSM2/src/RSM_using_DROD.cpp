#include "RSM_using_DROD.h"

// 构造函数
RSM_using_DROD_::RSM_using_DROD_()
    : nh("~"),
      tag_x_real(0.0),
      tag_y_real(0.0),
      tag_z_real(0.0),
      drod_x(nh),
      drod_y(nh),
      drod_z(nh),
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
    relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &RSM_using_DROD_::relative_pos_Callback, this, ros::TransportHints().tcpNoDelay());
    Odom_sub = nh.subscribe("/mavros/local_position/odom", 1, &RSM_using_DROD_::Odom_Callback, this, ros::TransportHints().tcpNoDelay());
    timer = nh.createTimer(ros::Duration(filter_for_x.dt), &RSM_using_DROD_::timerCallback, this);
    hat_pub = nh.advertise<std_msgs::Float64MultiArray>("/hat_error_xyz", 1);
}

void RSM_using_DROD_::timerCallback(const ros::TimerEvent &)
{
    function(loss_or_not_); // 立即执行一次
    timer_count++;          // 增加计数
    if (is_data_refreshed)
    {
        timer_count = 0;
        is_data_refreshed = false;
    }
}

void RSM_using_DROD_::Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_x = msg->pose.pose.position.x;
    uav_y = msg->pose.pose.position.y;
    uav_z = msg->pose.pose.position.z;
    uav_vx = msg->twist.twist.linear.x;
    uav_vy = msg->twist.twist.linear.y;
    uav_vz = msg->twist.twist.linear.z;
    uav_x = 0;
    uav_y = 0;
    uav_z = 0;
    uav_vx = 0;
    uav_vy = 0;
    uav_vz = 0;
}

void RSM_using_DROD_::function(bool loss_or_not_)
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
        drod_x.resetVectors();
        drod_y.resetVectors();
        drod_z.resetVectors();
        drod_x.resetVectors_past();
        drod_y.resetVectors_past();
        drod_z.resetVectors_past();
        drod_x.z_future[0](0) = tag_x_real;
        drod_x.z_future[0](1) = 0;
        filter_for_x.x_hat << tag_x_real, 0, 0;
        drod_y.z_future[0](0) = tag_y_real;
        drod_y.z_future[0](1) = 0;
        filter_for_y.x_hat << tag_y_real, 0, 0;
        drod_z.z_future[0](0) = tag_z_real;
        drod_z.z_future[0](1) = 0;
        filter_for_z.x_hat << tag_z_real, 0, 0;
    }
    else
    {
        if (timer_count == 0)
        {
            is_data_refreshed = false;
            drod_x.run(tag_x_real);
            Vector3d x_measure;
            x_measure << tag_x_real, drod_x.z_future[1];
            filter_for_x.predict();
            filter_for_x.updateJoint(x_measure);
            drod_y.run(tag_y_real);
            Vector3d y_measure;
            y_measure << tag_y_real, drod_y.z_future[1];
            filter_for_y.predict();
            filter_for_y.updateJoint(y_measure);
            drod_z.run(tag_z_real);
            Vector3d z_measure;
            z_measure << tag_z_real, drod_z.z_future[1];
            filter_for_z.predict();
            filter_for_z.updateJoint(z_measure);
        }
        else
        {
            int count = timer_count * static_cast<int>(filter_for_x.dt / drod_x.T_c);
            if (count > drod_x.Np)
            {
                count = drod_x.Np - 1;
            }
            filter_for_x.predict();
            filter_for_x.updateH2(drod_x.z_future[count]);
            filter_for_y.predict();
            filter_for_y.updateH2(drod_y.z_future[count]);
            filter_for_z.predict();
            filter_for_z.updateH2(drod_z.z_future[count]);
        }
    }
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(filter_for_x.x_hat(0) - uav_x);
    msg.data.push_back(filter_for_x.x_hat(1) - uav_vx);
    msg.data.push_back(filter_for_y.x_hat(0) - uav_y);
    msg.data.push_back(filter_for_y.x_hat(1) - uav_vy);
    msg.data.push_back(filter_for_z.x_hat(0) - uav_z);
    msg.data.push_back(filter_for_z.x_hat(1) - uav_vz);
    msg.data.push_back(tag_x_real - uav_x);
    msg.data.push_back(tag_y_real - uav_y);
    msg.data.push_back(tag_z_real - uav_z);
    msg.data.push_back(loss_or_not_);
    msg.data.push_back(des_yaw);
    hat_pub.publish(msg);
}

void RSM_using_DROD_::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
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
    ros::init(argc, argv, "rsm_using_drod");
    RSM_using_DROD_ rsm_using_drod;
    rsm_using_drod.spin();
    return 0;
}
