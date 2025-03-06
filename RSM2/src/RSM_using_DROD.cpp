#include "RSM_using_DROD.h"

// 构造函数
RSM_using_DROD_::RSM_using_DROD_()
    : nh("~"),
      tag_x_real(0.0),
      tag_y_real(0.0),
      tag_z_real(0.0),
      drod(nh, 0.06, 0.058, 0.02),
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
    timer = nh.createTimer(ros::Duration(filter.dt), &RSM_using_DROD_::timerCallback, this);
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
    double start_time = ros::Time::now().toSec();
    if (loss_or_not_ && loss_target == false) // 从能看到目标到看不到目标
    {
        loss_target = true;
    }
    if (!loss_or_not_ && loss_target == true) // 从不能看到目标到能看到目标
    {
        loss_target = false;
        first_time_in_fun = true;
    }
    if (first_time_in_fun || timer_count > 2)
    {
        first_time_in_fun = false;
        drod.resetVectors();
        drod.resetVectors_past();
        drod.z_future[0](0) = tag_x_real;
        drod.z_future[0](1) = 0;
        drod.z_future[0](2) = tag_y_real;
        drod.z_future[0](3) = 0;
        drod.z_future[0](4) = tag_z_real;
        drod.z_future[0](5) = 0;
        drod.z_future_dt[0](0) = tag_x_real;
        drod.z_future_dt[0](1) = 0;
        drod.z_future_dt[0](2) = tag_y_real;
        drod.z_future_dt[0](3) = 0;
        drod.z_future_dt[0](4) = tag_z_real;
        drod.z_future_dt[0](5) = 0;
        filter.x_hat << tag_x_real, 0, 0, tag_y_real, 0, 0, tag_z_real, 0, 0;
    }
    else
    {
        if (timer_count == 0)
        {
            int Np = drod.N_p;
            is_data_refreshed = false;
            filter.predict();
            Vector6d state_pre_drod;
            state_pre_drod << drod.z_future_dt[Np](0), drod.z_future_dt[Np](1), drod.z_future_dt[Np](2),
                drod.z_future_dt[Np](3), drod.z_future_dt[Np](4), drod.z_future_dt[Np](5);
            filter.updateH2(state_pre_drod);
            filter_delay.P = filter.P;
            filter_delay.x_hat = filter.x_hat;
            Vector9d All_measure;
            All_measure << Measure_xyz, drod.z_future_dt[0](0), drod.z_future_dt[0](1), drod.z_future_dt[0](2),
                drod.z_future_dt[0](3), drod.z_future_dt[0](4), drod.z_future_dt[0](5);
            filter_delay.updateJoint(All_measure);
            for (int i = 0; i < 3; i++)
            {
                filter_delay.predict();
                Vector6d state_pre_drod_temp;
                state_pre_drod_temp << drod.z_future_dt[i + 1](0), drod.z_future_dt[i + 1](1), drod.z_future_dt[i + 1](2),
                    drod.z_future_dt[i + 1](3), drod.z_future_dt[i + 1](4), drod.z_future_dt[i + 1](5);
                filter_delay.updateH2(state_pre_drod_temp);
            }
            filter.P = filter_delay.P;
            filter.x_hat = filter_delay.x_hat;
            drod.run(Measure_xyz);
            cout << "        " << endl;
            cout << filter.x_hat(3) - des_yaw << endl;
            // cout << drod_x.z_future[1](0) - des_yaw << endl;
        }
        else
        {
            if (timer_count <= 2)
            {
                filter.predict();
                Vector6d state_pre_drod_temp;
                state_pre_drod_temp << drod.z_future_dt[timer_count](0), drod.z_future_dt[timer_count](1), drod.z_future_dt[timer_count](2),
                    drod.z_future_dt[timer_count](3), drod.z_future_dt[timer_count](4), drod.z_future_dt[timer_count](5);
                filter.updateH2(state_pre_drod_temp);
            }
        }
    }
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(filter.x_hat(0) - uav_x);
    msg.data.push_back(filter.x_hat(1) - uav_vx);
    msg.data.push_back(filter.x_hat(3) - uav_y);
    msg.data.push_back(filter.x_hat(4) - uav_vy);
    msg.data.push_back(filter.x_hat(6) - uav_z);
    msg.data.push_back(filter.x_hat(7) - uav_vz);
    msg.data.push_back(tag_x_real - uav_x);
    msg.data.push_back(tag_y_real - uav_y);
    msg.data.push_back(tag_z_real - uav_z);
    msg.data.push_back(loss_or_not_);
    msg.data.push_back(des_yaw);
    hat_pub.publish(msg);
    double end_time = ros::Time::now().toSec();
    cout << "time cost:" << end_time - start_time << endl;
}

void RSM_using_DROD_::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    tag_x_real = msg->data[0] + uav_x;
    tag_y_real = msg->data[1] + uav_y;
    tag_z_real = msg->data[2] + uav_z;
    Measure_xyz << tag_x_real, tag_y_real, tag_z_real;
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
