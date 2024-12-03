#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
using namespace std;
using namespace Eigen;

// 定义Kalman滤波器类
class KalmanFilter
{
public:
    double dt = 0.02;
    // 状态向量维度
    static const int STATE_DIM = 3;
    // 最大测量向量维度
    static const int MAX_MEAS_DIM = 3; // y1 + y2

    // 状态转移矩阵 A
    Matrix<double, STATE_DIM, STATE_DIM> A;
    // 过程噪声协方差矩阵 Q
    Matrix<double, STATE_DIM, STATE_DIM> Q;
    // 测量矩阵 H1 和 H2
    Matrix<double, 1, STATE_DIM> H1;
    Matrix<double, 2, STATE_DIM> H2;
    // 联合测量矩阵 H_joint
    Matrix<double, 3, STATE_DIM> H_joint;
    // 测量噪声协方差矩阵 R1 和 R2
    Matrix<double, 1, 1> R1;
    Matrix<double, 2, 2> R2;
    // 联合测量噪声协方差矩阵 R_joint
    Matrix<double, 3, 3> R_joint;

    // 状态估计和误差协方差矩阵
    Vector3d x_hat;
    Matrix<double, STATE_DIM, STATE_DIM> P;

    // 构造函数
    KalmanFilter()
    {
        // 初始化系统矩阵 A
        A << 1, dt, 0,
            0, 1, dt,
            0, 0, 1;

        // 初始化测量矩阵 H1 和 H2
        H1 << 1, 0, 0;
        H2 << 1, 0, 0,
            0, 1, 0;

        // 初始化联合测量矩阵 H_joint
        H_joint << 1, 0 , 0,
            1, 0, 0,
            0, 1, 0;

        // 初始化过程噪声协方差 Q（根据实际情况调整）
        Q << 1e-4, 0, 0,
            0, 1e-3, 0,
            0, 0, 1e-2; 

        // 初始化测量噪声协方差 R1 和 R2（根据实际情况调整）
        R1 << 1e-4; // 慢测量位置噪声较小
        R2 << 2e-4, 0,
            0, 1e-3; // 快测量位置和速度噪声较大

        // 初始化联合测量噪声协方差矩阵 R_joint
        R_joint = Matrix<double, 3, 3>::Zero();
        R_joint(0, 0) = R1(0, 0);       // y1的噪声
        R_joint.block<2, 2>(1, 1) = R2; // y2的噪声

        // 初始化状态估计 x_hat（根据初始条件设定）
        x_hat << 0, 0, 0;

        // 初始化误差协方差矩阵 P
        P = Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    }

    // 预测步骤
    void predict()
    {
        // 状态预测
        x_hat = A * x_hat;
        // 误差协方差预测
        P = A * P * A.transpose() + Q;
    }

    // 更新步骤（使用H1和H2一起）
    void updateJoint(const Vector3d &y_joint)
    {
        // 计算卡尔曼增益
        Matrix<double, STATE_DIM, 3> K = P * H_joint.transpose() *
                                         (H_joint * P * H_joint.transpose() + R_joint).inverse();
        // 更新状态估计
        x_hat = x_hat + K * (y_joint - H_joint * x_hat);
        // 更新误差协方差矩阵
        P = (Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K * H_joint) * P;
    }

    // 更新步骤（仅使用H2和R2）
    void updateH2(const Vector2d &y2)
    {
        // 计算卡尔曼增益
        Matrix<double, STATE_DIM, 2> K = P * H2.transpose() *
                                         (H2 * P * H2.transpose() + R2).inverse();
        // 更新状态估计
        x_hat = x_hat + K * (y2 - H2 * x_hat);
        // 更新误差协方差矩阵
        P = (Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K * H2) * P;
    }
};

class APO
{
public:
    // parameters and variables for the APO
    Eigen::Vector2d hat_tag_x, hat_tag_y, hat_tag_z, C_bar;
    Eigen::Matrix2d A_bar, A0;
    double predict_tag_x, tag_x_real, predict_tag_y, tag_y_real, predict_tag_z, tag_z_real;
    double uav_x, uav_y, uav_z, uav_vx, uav_vy, uav_vz;
    double des_yaw;

    int timer_count;
    ros::Timer timer;

    // variables for control logic
    bool first_time_in_fun, loss_target, loss_or_not_;
    bool is_data_refreshed = false;

    // ros related
    ros::NodeHandle nh;
    ros::Subscriber relative_pos_sub;
    ros::Subscriber Odom_sub;
    ros::Publisher hat_pub;

    // Kalman filter
    KalmanFilter filter_for_x;
    KalmanFilter filter_for_y;
    KalmanFilter filter_for_z;

    // functions
    APO();
    void timerCallback(const ros::TimerEvent &);
    void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg);
    void function(bool loss_or_not);
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};
