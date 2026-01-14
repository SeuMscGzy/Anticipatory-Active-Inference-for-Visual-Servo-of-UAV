#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "aai_qpOASES.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <vector>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <atomic>
#include <mavros_msgs/ESCStatus.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;
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

class APO
{
private:
    // 你原来的成员...
    Eigen::Vector2d hat_x_last, hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double predict_y, y_real;
    bool first_time_in_fun, loss_target;
    double loss_or_not_;
    double u;
    LowPassFilter filter_for_img;

    // 保护测量缓存的互斥量（用于跨线程访问）
    std::mutex mtx_;

    // 测量缓存 & 标志
    bool has_new_measurement_;
    double latest_measure_;
    double latest_loss_or_not_;
    friend class AAI_DA;

public:
    double u_thr;
    APO()
        : hat_x_last(Eigen::Vector2d::Zero()),
          hat_x(Eigen::Vector2d::Zero()),
          predict_y(0.0),
          y_real(0.0),
          filter_for_img(0.9),
          loss_target(true),
          loss_or_not_(1),
          u(0.0),
          u_thr(0.0),
          first_time_in_fun(true),
          has_new_measurement_(false)
    {
        A_bar << 0.200723035694621, 0.0133815357129748,
            -8.36345982060922, 0.869799821343359;
        B_bar << 0, 0;
        C_bar << 0.799276964305379,
            8.36345982060923;
        A0 << 1, 0.0250000000000000,
            0, 1;
        B0 << 0.000312500000000000,
            0.0250000000000000;
    }

    void pushMeasurement(double measure_single_axis, double loss_or_not)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_measure_ = measure_single_axis;
        latest_loss_or_not_ = loss_or_not;
        has_new_measurement_ = true; // 告诉 step(): 下一次是采样步
    }

    void step()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // 1. 如果还没任何测量，就啥都不做（避免用垃圾初值）
        if (!has_new_measurement_ && first_time_in_fun)
        {
            return;
        }

        bool meas_this_step = false;

        // 2. 如果这一周期内来了新图像，就做滤波 & 标记为采样步
        if (has_new_measurement_)
        {
            // y_real = filter_for_img.filter(latest_measure_);
            y_real = latest_measure_;
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
            hat_x(0) = y_real;
        }
        else
        {
            if (meas_this_step)
            {
                // ====== 采样步（有新图像）======
                // 对应你之前的 timer_count == 0 分支
                predict_y = y_real;
                hat_x = A_bar * hat_x_last + B0 * u_thr + C_bar * predict_y;
            }
            else
            {
                // ====== 预测步（无新图像）======
                // 对应你之前的 timer_count > 0 分支
                Eigen::Vector2d coeff(1, 0);
                predict_y = coeff.transpose() * (A0 * hat_x_last + B0 * u_thr);
                hat_x = A_bar * hat_x_last + B_bar * u_thr + C_bar * predict_y;
            }
        }
        // 5. 更新 hat_x_last
        hat_x_last = hat_x;
    }
};

class AAI_DA
{
public:
    // parameters and variables for the optimization problem
    double dt = 0.025;
    int Np = 30;
    double precice_z1 = 2;
    double precice_z2 = 0.5;
    double precice_w1 = 2;
    double precice_w2 = 1;
    double e1 = 1;
    double precice_z_u = 0.021;
    double umin = -5;
    double umax = 5;
    Optimizer optimizer_xyz;
    std::array<double, 10> optical_xyz{};
    std::atomic<double> ground_truth_x, ground_truth_y, ground_truth_z;
    std::atomic<double> ground_truth_first_deri_x, ground_truth_first_deri_y, ground_truth_first_deri_z;
    std::atomic<double> ground_truth_x_car, ground_truth_y_car, ground_truth_z_car;
    std::atomic<double> ground_truth_first_deri_x_car, ground_truth_first_deri_y_car, ground_truth_first_deri_z_car;

    APO APOX, APOY, APOZ;
    double img_x, img_y, img_z = 0;
    bool lost = true;
    int32_t motor1, motor2, motor3, motor4;
    double q_w, q_x, q_y, q_z;
    double T_total;
    Eigen::Matrix3d R_world_from_body = Eigen::Matrix3d::Identity();
    // parameters and variables for the controller
    std::atomic<double> des_yaw;
    quadrotor_msgs::PositionCommand acc_msg;

    // ros related
    ros::Publisher acc_cmd_pub;
    ros::Subscriber relative_pos_sub;
    ros::Subscriber ground_truth_sub;
    ros::Subscriber ground_truth_pose_sub;
    ros::Subscriber ground_truth_sub_car;
    ros::Subscriber ground_truth_pose_sub_car;
    ros::Subscriber motor_speed_sub;
    ros::Subscriber odom_sub_;
    ros::Publisher pub_hat_x;

    ros::Timer excel_update_timer;

    // 控制线程
    std::thread xyz_thread;

    // functions
    AAI_DA(ros::NodeHandle &nh);
    ~AAI_DA();
    void startControlLoops();
    void xyzAxisControlLoop();
    void odomCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void ground_truth_callback_car(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void escStatusCb(const mavros_msgs::ESCStatus::ConstPtr &msg);
    void ground_truth_pose_callback_car(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void controlUpdate(const ros::TimerEvent &);
};