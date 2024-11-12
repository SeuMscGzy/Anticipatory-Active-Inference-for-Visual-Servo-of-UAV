#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "mpai_qp.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <vector>
using namespace std;
using namespace Eigen;
class RAID_AgiVS
{
public:
    // parameters and variables for the optimization problem
    double dt = 0.01;
    int Np = 40;
    double precice_z1 = 2;
    double precice_z2 = 1;
    double precice_w1 = 2;
    double precice_w2 = 2;
    double precice_z_u = 0.003;
    double e1 = 2;
    double umin = -4;
    double umax = 4;
    vector<double> optical_solution = {0.0, 100.0, 0.0, 0.0};
    Optimizer optimizer;

    // parameters and variables for the APO
    Eigen::Vector2d hat_x, B_bar, C_bar, B0;
    Eigen::Matrix2d A_bar, A0;
    double predict_y, y_real, y_speed_real;

    // parameters and variables for the controller
    double u;
    int timer_count;
    ros::Timer timer;
    bool run_control_loop;
    bool first_in_callback = true;

    // ros related
    ros::NodeHandle nh;
    ros::Publisher u_pub;
    ros::Subscriber relative_pos_sub;
    ros::Publisher pub_hat_x;

    // functions
    RAID_AgiVS(ros::NodeHandle &nh);
    void cal_single_axis_ctrl_input();
    void timerCallback(const ros::TimerEvent &);
    void function();
    void relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
};