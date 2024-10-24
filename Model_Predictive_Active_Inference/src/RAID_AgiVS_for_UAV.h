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
    Optimizer optimizer_x, optimizer_y, optimizer_z;
    double dt = 0.01;
    int Np = 100;
    double precice_z1 = 4;
    double precice_z2 = 0.1;
    double precice_w1 = 2;
    double precice_w2 = 2;
    double e1 = 3;
    double precice_z_u = 0.005;
    double umin = -4;
    double umax = 4;

    // parameters and variables for the APO
    Eigen::Vector2d hat_x_last, hat_x, hat_y_last, hat_y, hat_z_last, hat_z, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double predict_x, x_real;
    double predict_y, y_real;
    double predict_z, z_real;

    // parameters and variables for the controller
    vector<double> optical_x, optical_y, optical_z = {0, 0, 0, 0};
    double u_x, u_y, u_z;
    int count, timer_count;
    ros::Timer timer;
    std_msgs::Float64MultiArray relative_pos;

    // variables for control logic
    bool first_time_in_fun, loss_target, use_bias_, loss_or_not_;
    int px4_state;
    bool land_or_just_tracking = 0;
    bool keep_in_land = false;

    // ros related
    ros::NodeHandle nh;
    ros::Subscriber px4_state_sub;
    ros::Subscriber relative_pos_sub;
    ros::Publisher pub_land;

    // functions
    RAID_AgiVS();
    void cal_optical_ctrl();
    void cal_ctrl_input(double loss_or_not, bool use_bias);
    void timerCallback(const ros::TimerEvent &);
    void function(bool loss_or_not);
    void StateCallback(const std_msgs::Int32::ConstPtr &msg);
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    double adjustBias(double value, int which_axis, bool use_bias);
    void spin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
};