#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <Iir.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
using namespace std;

class AIC2Controller
{
private:
    const double e_1 = 1;
    const double e_2 = 1;
    const double k_l = 1.0;
    const double sigma_z1_inv = 1.0;
    const double sigma_z2_inv = 1.0;
    const double sigma_w1_inv = 1.0;
    const double sigma_w2_inv = 1.0;
    double k_i = -0.2;
    double k_p = -4.7;
    double k_d = -3;
    const double T_c = 0.02;
    double x_bias = 0;
    double y_bias = 0;
    double z_bias = 0.8;
    double y1_APO_fast_bias = 0;
    ros::NodeHandle nh;

public:
    AIC2Controller() // 构造函数
    {
    }

    double adjustBias(double value, int which_axis, bool use_bias) // which_axis: 0 for x, 1 for y, 2 for z
    {
        if (use_bias)
        {
            if (which_axis == 0)
            {
                return value + x_bias;
            }
            else if (which_axis == 1)
            {
                return value + y_bias;
            }
            else
            {
                return value + z_bias;
            }
        }
        else
        {
            return value;
        }
    }

    double limitControl(double controlValue)
    {
        if (abs(controlValue) >= 5)
        {
            controlValue = 5 * controlValue / abs(controlValue); // Apply limit
        }
        return controlValue;
    }

    double limitIntegral(double IntegralValue)
    {
        if (abs(IntegralValue) >= 1)
        {
            IntegralValue = IntegralValue / abs(IntegralValue); // Apply limit
        }
        return IntegralValue;
    }

    void computeControl(double y1_APO_fast, double y2_APO_fast, double &mu_last, double &mu_p_last, double &u_inte, double &u, double &mu, double &mu_p, bool use_bias, int which_axis)
    {
        y1_APO_fast_bias = adjustBias(y1_APO_fast, which_axis, use_bias);
        mu = double(mu_last + T_c * (mu_p_last + k_l * sigma_z1_inv * (y1_APO_fast_bias - mu_last) - k_l * sigma_w1_inv * e_1 * (mu_p_last + e_1 * mu_last)));
        mu_p = double(mu_p_last + T_c * (k_l * sigma_z2_inv * (y2_APO_fast - mu_p_last) - k_l * sigma_w1_inv * (mu_p_last + e_1 * mu_last) - k_l * sigma_w2_inv * e_2 * e_2 * mu_p_last));
        mu_last = mu;
        mu_p_last = mu_p;
        u_inte = double(u_inte - T_c * (k_i * (y1_APO_fast_bias - mu)));
        u_inte = limitIntegral(u_inte);
        u = u_inte - k_d * y2_APO_fast - k_p * (y1_APO_fast_bias - mu);
        u = limitControl(u);
    }
};

class LowPassFilter
{
private:
    double alpha;
    double last_output;

public:
    // Constructor
    LowPassFilter(double alpha_value) : alpha(alpha_value), last_output(0.0) {}

    // Filter a new value
    double filter(double input)
    {
        double output = alpha * input + (1 - alpha) * last_output;
        last_output = output;
        return output;
    }
};

class MyController
{
private:
    // 你原来的成员...
    Eigen::Vector2d hat_x_last, hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double u, u_inte, predict_y, y_real;
    double mu, mu_p, mu_last, mu_p_last;
    bool first_time_in_fun, loss_target, use_bias_;
    int which_axis_;
    double loss_or_not_;
    LowPassFilter filter_for_img;
    AIC2Controller aic2controller;
    ros::NodeHandle nh;

    // 新增：测量缓存 & 标志
    bool has_new_measurement_;
    double latest_measure_;
    double latest_loss_or_not_;
    friend class TripleAxisController;

public:
    MyController()
        : hat_x_last(Eigen::Vector2d::Zero()),
          nh("~"),
          hat_x(Eigen::Vector2d::Zero()),
          predict_y(0.0),
          y_real(0.0),
          u(0.0),
          u_inte(0.0),
          mu(0.0),
          mu_p(0.0),
          mu_last(0.0),
          mu_p_last(0.0),
          filter_for_img(0.95),
          loss_target(true),
          loss_or_not_(1),
          use_bias_(1),
          which_axis_(0),
          first_time_in_fun(true),
          has_new_measurement_(false)
    {
        A_bar << 0.340893168573740, 0.0126256729101385,
            -6.67898096946328, 0.921674122440112;
        B_bar << 0, 0;
        C_bar << 0.659106831426260,
            6.67898096946328;
        A0 << 1, 0.02,
            0, 1;
        B0 << 0.0002,
            0.02;
    }

    void pushMeasurement(double measure_single_axis,
                         double loss_or_not,
                         bool use_bias,
                         int which_axis)
    {
        latest_measure_ = measure_single_axis;
        latest_loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        has_new_measurement_ = true; // 告诉 step(): 下一次是采样步
    }

    void step()
    {
        // 1. 如果还没任何测量，就啥都不做（避免用垃圾初值）
        if (!has_new_measurement_ && first_time_in_fun)
        {
            return;
        }

        bool meas_this_step = false;

        // 2. 如果这一周期内来了新图像，就做滤波 & 标记为采样步
        if (has_new_measurement_)
        {
            y_real = filter_for_img.filter(latest_measure_);
            loss_or_not_ = latest_loss_or_not_;
            has_new_measurement_ = false;
            meas_this_step = true;
        }

        // 3. 用 loss 标志更新 loss_target / first_time 状态
        if (loss_or_not_ == 1 && loss_target == false) // 从看到目标 -> 丢失
        {
            loss_target = true;
        }
        if (loss_or_not_ == 0 && loss_target == true) // 从丢失 -> 重新看到
        {
            loss_target = false;
            first_time_in_fun = true;
        }

        // 4. 真正的 APO + AIC2 更新逻辑
        if (first_time_in_fun && meas_this_step)
        {
            // 第一次有测量的那一步：初始化
            first_time_in_fun = false;
            predict_y = y_real;
            hat_x(0) = y_real;
            // 这里相当于你原来 first_time 分支
            aic2controller.computeControl(hat_x(0), hat_x(1), mu_last, mu_p_last, u_inte, u, mu, mu_p, use_bias_, which_axis_);
        }
        else
        {
            if (meas_this_step)
            {
                // ====== 采样步（有新图像）======
                // 对应你之前的 timer_count == 0 分支
                predict_y = y_real;
                u = 0;
                hat_x = A_bar * hat_x_last + B0 * u + C_bar * predict_y;
                aic2controller.computeControl(hat_x(0), hat_x(1), mu_last, mu_p_last, u_inte, u, mu, mu_p, use_bias_, which_axis_);
            }
            else
            {
                // ====== 预测步（无新图像）======
                // 对应你之前的 timer_count > 0 分支
                Eigen::Vector2d coeff(1, 0);
                u = 0;
                predict_y = coeff.transpose() * (A0 * hat_x_last + B0 * u);
                hat_x = A_bar * hat_x_last + B_bar * u + C_bar * predict_y;
                aic2controller.computeControl(hat_x(0), hat_x(1), mu_last, mu_p_last, u_inte, u, mu, mu_p, use_bias_, which_axis_);
            }
        }

        // 5. 更新 hat_x_last
        hat_x_last = hat_x;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data != 3)
        {
            u_inte = 0;
        }
    }
};

class TripleAxisController
{
private:
    MyController controllerX, controllerY, controllerZ;
    ros::NodeHandle nh;
    ros::Subscriber relative_position_sub, ground_truth_sub, ground_truth_pose_sub, px4_state_sub;
    ros::Subscriber ground_truth_sub_car, ground_truth_pose_sub_car;
    ros::Publisher pub_hat_x, acc_cmd_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    double des_yaw;
    ros::Timer control_update_timer;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_first_deri_x_car = 0, ground_truth_first_deri_y_car = 0, ground_truth_first_deri_z_car = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;
    double ground_truth_x_car = 0, ground_truth_y_car = 0, ground_truth_z_car = 0;

public:
    TripleAxisController()
        : nh("~"), des_yaw(0)
    {
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &TripleAxisController::px4StateCallback, this);
        relative_position_sub = nh.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this, ros::TransportHints().tcpNoDelay());
        ground_truth_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &TripleAxisController::ground_truth_callback, this);
        ground_truth_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        ground_truth_sub_car = nh.subscribe("/vrpn_client_node/Tracker0/0/twist", 10, &TripleAxisController::ground_truth_callback_car, this);
        ground_truth_pose_sub_car = nh.subscribe("/vrpn_client_node/Tracker0/0/pose", 10, &TripleAxisController::ground_truth_pose_callback_car, this);
        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        control_update_timer = nh.createTimer(ros::Duration(0.02), &TripleAxisController::controlUpdate, this);
    }

    void px4StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        controllerX.StateCallback(msg);
        controllerY.StateCallback(msg);
        controllerZ.StateCallback(msg);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        des_yaw = msg->data[5];

        controllerX.pushMeasurement(msg->data[0], msg->data[4], 0, 0);
        controllerY.pushMeasurement(msg->data[1], msg->data[4], 0, 1);
        controllerZ.pushMeasurement(msg->data[2], msg->data[4], 1, 2);
    }

    void ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        ground_truth_first_deri_x = msg->twist.linear.x;
        ground_truth_first_deri_y = msg->twist.linear.y;
        ground_truth_first_deri_z = msg->twist.linear.z;
    }

    void ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ground_truth_x = msg->pose.position.x;
        ground_truth_y = msg->pose.position.y;
        ground_truth_z = msg->pose.position.z;
    }

    void ground_truth_callback_car(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        ground_truth_first_deri_x_car = msg->twist.linear.x;
        ground_truth_first_deri_y_car = msg->twist.linear.y;
        ground_truth_first_deri_z_car = msg->twist.linear.z;
    }

    void ground_truth_pose_callback_car(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ground_truth_x_car = msg->pose.position.x;
        ground_truth_y_car = msg->pose.position.y;
        ground_truth_z_car = msg->pose.position.z;
    }

    void controlUpdate(const ros::TimerEvent &)
    {
        // 1. 先更新三个轴的控制（严格 50 Hz）
        controllerX.step();
        controllerY.step();
        controllerZ.step();
        acc_msg.position.x = 0;
        acc_msg.position.y = 0;
        acc_msg.position.z = 0;
        acc_msg.velocity.x = 0;
        acc_msg.velocity.y = 0;
        acc_msg.velocity.z = 0;
        acc_msg.acceleration.x = controllerX.u;
        acc_msg.acceleration.y = controllerY.u;
        acc_msg.acceleration.z = controllerZ.u;
        acc_msg.jerk.x = 0;
        acc_msg.jerk.y = 0;
        acc_msg.jerk.z = 0;
        acc_msg.yaw = des_yaw;
        acc_msg.yaw_dot = 0;
        acc_msg.header.frame_id = "world";
        acc_msg.header.stamp = ros::Time::now();
        acc_cmd_pub.publish(acc_msg);
        std_msgs::Float64MultiArray msg1;
        msg1.data.resize(24);

        for (int i = 0; i < 2; i++)
        {
            msg1.data[i] = controllerX.hat_x(i);
        }
        for (int i = 8; i < 10; i++)
        {
            msg1.data[i] = controllerY.hat_x(i - 6);
        }
        for (int i = 16; i < 18; i++)
        {
            msg1.data[i] = controllerZ.hat_x(i - 12);
        }
        msg1.data[2] = controllerX.u;
        msg1.data[3] = controllerX.y_real;
        msg1.data[4] = ground_truth_x;
        msg1.data[5] = ground_truth_first_deri_x;
        msg1.data[6] = ground_truth_x_car;
        msg1.data[7] = ground_truth_first_deri_x_car;

        msg1.data[10] = controllerY.u;
        msg1.data[11] = controllerY.y_real;
        msg1.data[12] = ground_truth_y;
        msg1.data[13] = ground_truth_first_deri_y;
        msg1.data[14] = ground_truth_y_car;
        msg1.data[15] = ground_truth_first_deri_y_car;

        msg1.data[18] = controllerZ.u;
        msg1.data[19] = controllerZ.y_real;
        msg1.data[20] = ground_truth_z;
        msg1.data[21] = ground_truth_first_deri_z;
        msg1.data[22] = ground_truth_z_car;
        msg1.data[23] = ground_truth_first_deri_z_car;
        pub_hat_x.publish(msg1);
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller_node");
    TripleAxisController controller;
    // TODO: Add ROS subscribers, publishers, and service servers/clients as needed
    controller.spin();
    return 0;
}
