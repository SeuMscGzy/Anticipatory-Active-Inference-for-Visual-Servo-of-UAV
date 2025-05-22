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
    const double T_c = 0.01;
    double x_bias = 0;
    double y_bias = 0;
    double z_bias = 1.5;
    double y1_APO_fast_bias = 0;
    double y1_real_bias = 0;
    ros::NodeHandle nh;
    std::vector<double> trust_array_y1 = {1, 0.75, 0.65, 0.55, 0.45};
    std::vector<double> trust_array_y2 = {0.5, 0.5, 0.5, 0.5, 0.5};

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

    double limitControl(double controlValue, int which_axis)
    {
        double limit = (which_axis != 0) ? 5 : 5; // Limit is 3 for y and z, 1.5 for x axis.
        if (abs(controlValue) >= limit)
        {
            controlValue = limit * controlValue / abs(controlValue); // Apply limit
        }
        return controlValue;
    }

    double limitIntegral(double IntegralValue, int which_axis)
    {
        double limit = (which_axis != 0) ? 1 : 1; // Limit is 1.5 for y and z, 0.75 for x axis.
        if (abs(IntegralValue) >= limit)
        {
            IntegralValue = limit * IntegralValue / abs(IntegralValue); // Apply limit
        }
        return IntegralValue;
    }

    void computeControl(int time_count, double y1_real_slow, double y1_APO_fast, double y2_derivative_sampling, double y2_APO_fast, double &mu_last, double &mu_p_last, double &u_last, double &u, double &mu, double &mu_p, bool use_bias, int which_axis)
    {
        y1_APO_fast_bias = adjustBias(y1_APO_fast, which_axis, use_bias);
        y1_real_bias = adjustBias(y1_real_slow, which_axis, use_bias);
        double trust_param_y1 = trust_array_y1[time_count];
        double trust_param_y2 = trust_array_y2[time_count];
        mu = double(mu_last + T_c * (mu_p_last + (1 - trust_param_y1) * k_l * sigma_z1_inv * (y1_APO_fast_bias - mu_last) + trust_param_y1 * k_l * sigma_z1_inv * (y1_real_bias - mu_last) - k_l * sigma_w1_inv * e_1 * (mu_p_last + e_1 * mu_last)));
        mu_p = double(mu_p_last + T_c * ((1 - trust_param_y2) * k_l * sigma_z2_inv * (y2_APO_fast - mu_p_last) + trust_param_y2 * k_l * sigma_z2_inv * (y2_derivative_sampling - mu_p_last) - k_l * sigma_w1_inv * (mu_p_last + e_1 * mu_last) - k_l * sigma_w2_inv * e_2 * e_2 * mu_p_last));
        mu_last = mu;
        mu_p_last = mu_p;
        u_last = double(u_last - T_c * (k_i * ((1 - trust_param_y1) * (y1_APO_fast_bias - mu) + trust_param_y1 * (y1_real_bias - mu))));
        u_last = limitIntegral(u_last, which_axis);
        u = u_last - k_d * (1 - trust_param_y2) * y2_APO_fast - k_d * trust_param_y2 * y2_derivative_sampling - k_p * ((1 - trust_param_y1) * (y1_APO_fast_bias - mu) + trust_param_y1 * (y1_real_bias - mu));
        u = limitControl(u, which_axis);
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

class ButterworthLowPassFilter
{
private:
    double fs; // sampling frequency
    double fc; // cut-off frequency
    double a0, a1, a2, b1, b2;
    double prevX1, prevX2, prevY1, prevY2;

public:
    ButterworthLowPassFilter(double sampleRate, double cutoffFrequency)
        : fs(sampleRate), fc(cutoffFrequency), prevX1(0), prevX2(0), prevY1(0), prevY2(0)
    {
        calculateCoefficients();
    }

    void calculateCoefficients()
    {
        double ita = 1.0 / tan(M_PI * fc / fs);
        double q = sqrt(2.0);
        double a0_temp = 1.0 / (1.0 + q * ita + ita * ita);
        a0 = a0_temp;
        a1 = 2 * a0_temp;
        a2 = a0_temp;
        b1 = 2.0 * a0_temp * (1 - ita * ita);
        b2 = a0_temp * (1 - q * ita + ita * ita);
    }

    double filter(double input)
    {
        double output = a0 * input + a1 * prevX1 + a2 * prevX2 - b1 * prevY1 - b2 * prevY2;
        prevX2 = prevX1;
        prevX1 = input;
        prevY2 = prevY1;
        prevY1 = output;
        return output;
    }

    void reset()
    {
        prevX1 = prevX2 = prevY1 = prevY2 = 0;
    }
};

class MyController
{
private:
    Eigen::Vector2d hat_x_last, hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double u;
    double u_last;
    double predict_y, y_real, y_real_last;
    double y_real_derivative, y_filtered_deri;
    double mu, mu_p, mu_last, mu_p_last;
    double time_now, time_last, time_pass;
    int count, timer_count;
    bool first_time_in_fun, loss_target, use_bias_;
    ros::NodeHandle nh;
    ros::Timer timer;

    // ButterworthLowPassFilter filter_for_deri; // 二阶巴特沃斯LPF
    LowPassFilter filter_for_img;
    AIC2Controller aic2controller;
    // Iir::Butterworth::LowPass<4> filter_4_for_img;
    Iir::Butterworth::LowPass<2> filter_4_for_deri;
    ros::Subscriber px4_state_sub;
    friend class TripleAxisController;
    double loss_or_not_;
    int which_axis_;
    int px4_state;

public:
    // 构造函数
    MyController()
        : hat_x_last(Eigen::Vector2d::Zero()),
          nh("~"),
          px4_state(1),
          hat_x(Eigen::Vector2d::Zero()),
          predict_y(0.0),
          y_real(0.0),
          u(0.0),
          u_last(0.0),
          y_real_last(0.0),
          y_real_derivative(0.0),
          time_now(0.0),
          time_last(0.0),
          time_pass(0.0),
          mu(0.0),
          mu_p(0.0),
          mu_last(0.0),
          mu_p_last(0.0),
          timer_count(0),
          filter_for_img(0.95),
          // filter_for_deri(20, 3),
          y_filtered_deri(0),
          loss_target(true),
          loss_or_not_(1),
          use_bias_(1),
          which_axis_(0),
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
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this, ros::TransportHints().tcpNoDelay());
        filter_4_for_deri.setup(20, 2.7);
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        y_real = measure_single_axis;
        y_real = filter_for_img.filter(y_real);
        // y_real = filter_4_for_img.filter(y_real);

        time_now = ros::Time::now().toSec();
        time_pass = time_now - time_last;
        function(loss_or_not_, use_bias_, which_axis_);

        y_real_derivative = (y_real - y_real_last) / time_pass; // 0.05 seconds = 50ms
        // y_filtered_deri = filter_for_deri.filter(y_real_derivative);
        y_filtered_deri = filter_4_for_deri.filter(y_real_derivative);

        // Update the last values for the next iteration
        y_real_last = y_real;
        timer_count++;
        timer = nh.createTimer(ros::Duration(0.01), &MyController::timerCallback, this);
        time_last = time_now;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        function(loss_or_not_, use_bias_, which_axis_);
        timer_count++;
        if (timer_count >= 5)
        {
            timer.stop();
            timer_count = 0;
        }
    }

    void function(double loss_or_not, bool use_bias, int which_axis)
    {
        if (loss_or_not == 1 && loss_target == false) // From seeing the target to not seeing the target
        {
            loss_target = true;
        }
        if (loss_or_not == 0 && loss_target == true) // From not seeing the target to seeing the target
        {
            loss_target = false;
            first_time_in_fun = true;
        }
        if (first_time_in_fun)
        {
            time_pass = 0.05;
            y_real_last = y_real;
            first_time_in_fun = false;
            predict_y = y_real;
            hat_x(0) = y_real;
            aic2controller.computeControl(timer_count, y_real, hat_x(0), y_filtered_deri, hat_x(1), mu_last, mu_p_last, u_last, u, mu, mu_p, use_bias, which_axis);
        }
        else
        {
            if (timer_count == 0)
            {
                predict_y = y_real;
                u = 0;
                hat_x = A_bar * hat_x_last + B0 * u + C_bar * predict_y;
                aic2controller.computeControl(timer_count, y_real, hat_x(0), y_filtered_deri, hat_x(1), mu_last, mu_p_last, u_last, u, mu, mu_p, use_bias, which_axis);
            }
            else
            {
                Eigen::Vector2d coeff(1, 0);
                u = 0;
                predict_y = coeff.transpose() * (A0 * hat_x_last + B0 * u);
                hat_x = A_bar * hat_x_last + B_bar * u + C_bar * predict_y;
                // hat_x = A_bar * hat_x_last + B_bar * u + C_bar * predict_y;
                aic2controller.computeControl(timer_count, y_real, hat_x(0), y_filtered_deri, hat_x(1), mu_last, mu_p_last, u_last, u, mu, mu_p, use_bias, which_axis);
            }
        }
        // Update last values for the next iteration
        hat_x_last = hat_x;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data != 3) // When not in cmd mode, the control amount is 0.
        {
            u = 0;
            u_last = 0;
            mu = 0;
            mu_p = 0;
            mu_last = 0;
            mu_p_last = 0;
        }
    }
};

class TripleAxisController
{
private:
    MyController controllerX, controllerY, controllerZ;
    ros::NodeHandle nh;
    ros::Subscriber relative_position_sub, ground_truth_sub, ground_truth_pose_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    double des_yaw;
    ros::Timer control_update_timer;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;

public:
    TripleAxisController()
        : nh("~"), des_yaw(0)
    {
        relative_position_sub = nh.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this, ros::TransportHints().tcpNoDelay());
        ground_truth_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &TripleAxisController::ground_truth_callback, this);
        ground_truth_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        control_update_timer = nh.createTimer(ros::Duration(0.01), &TripleAxisController::controlUpdate, this);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // Update the controller for each axis
        des_yaw = msg->data[5];
        controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], 1, 0);
        controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], 0, 1);
        controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], 1, 2); // Constant height tracking
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

    void controlUpdate(const ros::TimerEvent &)
    {
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
        acc_msg.header.frame_id = 'world';
        acc_cmd_pub.publish(acc_msg);
        std_msgs::Float64MultiArray msg1;
        msg1.data.resize(27);

        for (int i = 0; i < 2; i++)
        {
            msg1.data[i] = controllerX.hat_x(i);
        }
        for (int i = 9; i < 11; i++)
        {
            msg1.data[i] = controllerY.hat_x(i - 9);
        }
        for (int i = 18; i < 20; i++)
        {
            msg1.data[i] = controllerZ.hat_x(i - 18);
        }
        msg1.data[2] = controllerX.u;
        msg1.data[3] = controllerX.y_real;
        msg1.data[4] = -ground_truth_first_deri_x;
        msg1.data[5] = ground_truth_first_deri_x;
        msg1.data[6] = controllerX.y_filtered_deri;
        msg1.data[7] = ground_truth_first_deri_x;
        msg1.data[8] = ground_truth_x;

        msg1.data[11] = controllerY.u;
        msg1.data[12] = controllerY.y_real;
        msg1.data[13] = -ground_truth_first_deri_y;
        msg1.data[14] = ground_truth_first_deri_y;
        msg1.data[15] = controllerY.y_filtered_deri;
        msg1.data[16] = ground_truth_first_deri_y;
        msg1.data[17] = ground_truth_y;

        msg1.data[20] = controllerZ.u;
        msg1.data[21] = controllerZ.y_real;
        msg1.data[22] = -ground_truth_first_deri_z;
        msg1.data[23] = ground_truth_first_deri_z;
        msg1.data[24] = controllerZ.y_filtered_deri;
        msg1.data[25] = ground_truth_first_deri_z;
        msg1.data[26] = ground_truth_z;
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
