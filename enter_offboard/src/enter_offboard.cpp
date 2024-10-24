#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>


bool toggle_offboard_mode(bool on_off, ros::ServiceClient set_FCU_mode_srv)
{
    mavros_msgs::SetMode offb_set_mode;
    if (on_off)
    {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
        {
            ROS_ERROR("Enter OFFBOARD rejected by PX4!");
            return false;
        }
    }
    else
    {
        offb_set_mode.request.custom_mode = "MANUAL";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
        {
            ROS_ERROR("Exit OFFBOARD rejected by PX4!");
            return false;
        }
    }

    return true;

    // if (param.print_dbg)
    // 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    ros::ServiceClient set_FCU_mode_srv; // Declare the variable

    set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); // Assign a value to the variable
    ros::Duration(0.5).sleep();
    ros::Rate r(20);
    while (ros::ok())
    {
        int input;
        std::cout << "Enter 1 to toggle offboard mode ON, or 2 to toggle offboard mode OFF: ";
        std::cin >> input;

        if (input == 1) {
            toggle_offboard_mode(true, set_FCU_mode_srv);
        } else if (input == 2) {
            toggle_offboard_mode(false, set_FCU_mode_srv);
        }
        else if (input == 3) {
            break;
        }
         else {
            std::cout << "Invalid input. Please try again." << std::endl;
        }
        r.sleep();
        ros::spinOnce();
       // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}