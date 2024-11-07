#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;
class APO
{
public:
    // parameters and variables for the APO
    Eigen::Vector2d hat_tag_x, hat_tag_y, hat_tag_z, C_bar;
    Eigen::Matrix2d A_bar, A0;
    double predict_tag_x, tag_x_real, predict_tag_y, tag_y_real, predict_tag_z, tag_z_real;
    double uav_x, uav_y, uav_z, uav_vx, uav_vy, uav_vz;

    int timer_count;
    ros::Timer timer;
    std_msgs::Float64MultiArray relative_pos;

    // variables for control logic
    bool first_time_in_fun, loss_target, loss_or_not_;
    bool is_data_refreshed = false;

    // ros related
    ros::NodeHandle nh;
    ros::Subscriber relative_pos_sub;
    ros::Subscriber Odom_sub;
    ros::Publisher hat_pub;

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