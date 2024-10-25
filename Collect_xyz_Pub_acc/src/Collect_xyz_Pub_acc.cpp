#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

using namespace std;
using namespace Eigen;

class AccPublisher
{
public:
    AccPublisher()
        : nh("~"), px4_state(1), acc_x(0), acc_y(0), acc_z(0), des_yaw(0), land_or_just_tracking(false), keep_in_land(false)
    {
        // Initialize publishers
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        pub_land_ = nh.advertise<std_msgs::Bool>("/flight_land", 1);

        // Initialize subscribers
        px4_state_sub_ = nh.subscribe("/px4_state_pub", 1, &AccPublisher::State_Callback, this, ros::TransportHints().tcpNoDelay());
        acc_x_sub = nh.subscribe("/input_x_axis", 1, &AccPublisher::acc_x_Callback, this, ros::TransportHints().tcpNoDelay());
        acc_y_sub = nh.subscribe("/input_y_axis", 1, &AccPublisher::acc_y_Callback, this, ros::TransportHints().tcpNoDelay());
        acc_z_sub = nh.subscribe("/input_z_axis", 1, &AccPublisher::acc_z_Callback, this, ros::TransportHints().tcpNoDelay());
        relative_pos_sub_ = nh.subscribe("/point_with_fixed_delay", 1, &AccPublisher::relativepos_Callback, this, ros::TransportHints().tcpNoDelay());

        // Timer for control update
        control_update_timer = nh.createTimer(ros::Duration(0.01), &AccPublisher::controlUpdate, this);
    }

    void spin()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher acc_cmd_pub;
    ros::Publisher pub_land_;
    ros::Subscriber px4_state_sub_;
    ros::Subscriber acc_x_sub;
    ros::Subscriber acc_y_sub;
    ros::Subscriber acc_z_sub;
    ros::Subscriber relative_pos_sub_;
    ros::Timer control_update_timer;

    int px4_state;
    double acc_x;
    double acc_y;
    double acc_z;
    double des_yaw;
    bool land_or_just_tracking;
    bool keep_in_land;
    std_msgs::Float64MultiArray relative_pos_;
    quadrotor_msgs::PositionCommand acc_msg;

    void State_Callback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state = msg->data;
    }

    void acc_x_Callback(const std_msgs::Float64::ConstPtr &msg)
    {
        acc_x = msg->data;
    }

    void acc_y_Callback(const std_msgs::Float64::ConstPtr &msg)
    {
        acc_y = msg->data;
    }

    void acc_z_Callback(const std_msgs::Float64::ConstPtr &msg)
    {
        acc_z = msg->data;
    }

    void controlUpdate(const ros::TimerEvent &)
    {
        acc_msg.position.x = 0;
        acc_msg.position.y = 0;
        acc_msg.position.z = 0;
        acc_msg.velocity.x = 0;
        acc_msg.velocity.y = 0;
        acc_msg.velocity.z = 0;
        acc_msg.acceleration.x = acc_x;
        acc_msg.acceleration.y = acc_y;
        acc_msg.acceleration.z = acc_z;
        acc_msg.jerk.x = 0;
        acc_msg.jerk.y = 0;
        acc_msg.jerk.z = 0;
        acc_msg.yaw = des_yaw;
        acc_msg.yaw_dot = 0;
        acc_msg.header.frame_id = "world";
        acc_cmd_pub.publish(acc_msg);
    }

    void relativepos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        relative_pos_.data = msg->data;
        des_yaw = msg->data[5];
        if (land_or_just_tracking)
        {
            if (msg->data[0] > 0.15 || msg->data[1] > 0.15)
            {
                std_msgs::Bool land_msg;
                land_msg.data = false;
                pub_land_.publish(land_msg);
            }
            else
            {
                if ((msg->data[2] < 0.15 && msg->data[4] == 0 && px4_state == 3) || keep_in_land)
                {
                    // Send forced landing command as we might be losing sight of the target, open-loop close proximity landing
                    keep_in_land = true;
                    std_msgs::Bool land_msg;
                    land_msg.data = true;
                    pub_land_.publish(land_msg);
                    cout << "land message sent" << endl;
                }
            }
        }
        else
        {
            std_msgs::Bool land_msg;
            land_msg.data = false;
            pub_land_.publish(land_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_publisher");
    AccPublisher acc_publisher;
    acc_publisher.spin();
    return 0;
}
