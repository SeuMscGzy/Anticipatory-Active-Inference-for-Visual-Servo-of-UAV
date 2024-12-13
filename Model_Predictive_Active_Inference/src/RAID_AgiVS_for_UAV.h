#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "mpai_qp.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <vector>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
using namespace std;
using namespace Eigen;
class RAID_AgiVS
{
public:
    // parameters and variables for the optimization problem
    double dt = 0.02;
    int Np = 30;
    double precice_z1 = 2;
    double precice_z2 = 0.5;
    double precice_w1 = 2;
    double precice_w2 = 1;
    double e1 = 2;
    double precice_z_u = 0.01;
    double umin = -5;
    double umax = 5;
    Optimizer optimizer_xyz;
    vector<double> optical_xyz = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::atomic<double> x_real;
    std::atomic<double> xv_real;
    std::atomic<double> y_real;
    std::atomic<double> yv_real;
    std::atomic<double> z_real;
    std::atomic<double> zv_real;

    // parameters and variables for the controller
    std::atomic<double> des_yaw;
    quadrotor_msgs::PositionCommand acc_msg;

    // variables for control logic
    std::atomic<int> px4_state;
    bool land_or_just_tracking = false;
    bool keep_in_land = false;

    // ros related
    ros::NodeHandle nh;
    ros::Publisher acc_cmd_pub;
    ros::Publisher pub_land;
    ros::Subscriber px4_state_sub;
    ros::Subscriber relative_pos_sub;
    ros::Publisher pub_u;

    // 控制线程
    std::thread xyz_thread;

    // functions
    RAID_AgiVS(ros::NodeHandle &nh);
    ~RAID_AgiVS();
    void startControlLoops();
    void xyzAxisControlLoop();
    void cal_optical_ctrl_x();
    void cal_optical_ctrl_y();
    void cal_optical_ctrl_z();
    void uPubLoop();
    void StateCallback(const std_msgs::Int32::ConstPtr &msg);
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void adjustBias(bool use_bias);
};