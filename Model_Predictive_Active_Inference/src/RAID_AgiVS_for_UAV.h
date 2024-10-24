#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "mpai_qp.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
using namespace std;
using namespace Eigen;
class RAID_AgiVS
{
public:
    // parameters and variables for the optimization problem
    Optimizer optimizer_x;
    double dt = 0.01;
    int Np = 30;
    double precice_z1 = 2;
    double precice_z2 = 1;
    double precice_w1 = 2;
    double precice_w2 = 2;
    double e1 = 3;
    double precice_z_u = 0.005;
    double umin = -4;
    double umax = 4;

    // parameters and variables for the APO
    Eigen::Vector2d hat_x_last, hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double predict_x, x_real;

    // parameters and variables for the controller
    vector<double> optical_x = {0, 0, 0, 0};
    double u_x;
    double which_axis;
    int count, timer_count;
    ros::Timer timer;
    std_msgs::Float64MultiArray relative_pos;

    // variables for control logic
    bool first_time_in_fun, loss_target, loss_or_not_;
    int px4_state;
    bool land_or_just_tracking = false;
    bool run_control_loop = false;
    bool keep_in_land = false;

    // ros related
    ros::NodeHandle nh;
    ros::Subscriber px4_state_sub;
    ros::Subscriber relative_pos_sub;
    ros::Publisher pub_u;

    // functions
    RAID_AgiVS(int index);
    void cal_optical_ctrl();
    void timerCallback(const ros::TimerEvent &);
    void function(bool loss_or_not);
    void StateCallback(const std_msgs::Int32::ConstPtr &msg);
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    double adjustBias(double value, bool use_bias);
    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};