#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float64MultiArray.h>
#include "input.h"
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include "controller.h"
#include <std_msgs/Bool.h>

class PX4CtrlFSM
{
public:
	Parameter_t &param;
	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;

	LinearControl &controller;

	ros::Publisher ctrl_FCU_pub_att;
	ros::Publisher ctrl_FCU_pub_acc;
	ros::Publisher debug_pub; // debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	quadrotor_msgs::Px4ctrlDebug debug_msg; // debug

	Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;
	int loss_target_time_count;
	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER,		 // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL		 // px4ctrl is actived, and controling the drone.
	};

	PX4CtrlFSM(Parameter_t &, LinearControl &);
	void process();
	void loss_target_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	State_t get_state() { return state; }

private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	ros::NodeHandle nh;
	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();
	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm);		// It will only try to toggle once, so not blocked.
	void publish_acceleration_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
};

#endif