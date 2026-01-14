#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), nh("~"), loss_target_time_count(15), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
	// loss_target_sub = nh.subscribe("/point_with_unfixed_delay", 100, &PX4CtrlFSM::loss_taget_callback, this);
}

/*
		Finite State Machine

		  system start
				|
				|
				v
	----- > MANUAL_CTRL
	|         ^   |
	|         |   |
	|         |   |
	|         |   |
	|         |   |
	|         |   |
	|         |   v
	|       AUTO_HOVER
	|         ^   |
	|         |   |
	|         |	  |
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	u.thrust = 0;
	Desired_State_t des(odom_data); // des代表了当前的无人机位姿，利用初始化函数读取里程计信息来给其赋值

	// STEP1: state machine runs
	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER 这个变量是怎么赋值的？根据通道的信息来确定吗？
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			/*if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}*/
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			controller.resetThrustMapping();
			set_hov_with_odom(); // 使用里程计当前的信息进行悬停
			toggle_offboard_mode(true);
			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		break;
	}
	case AUTO_HOVER:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time) && loss_target_time_count == 0 && !takeoff_land_data.triggered && !in_landing)
		{
			// cout << "进了这里" << endl;
			if (state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				des = get_cmd_des(); // 自主控制需要改的函数
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		else // 悬停模式可以用遥控器来悬停，如果遥控器不操作的话就是保持最开始进入悬停时的位置和偏航角
		{
			// cout << "here" << endl;
			set_hov_with_rc();
			des = get_hover_des();
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}

			// cout << "des.p=" << des.p.transpose() << endl;
		}
		break;
	}

	case CMD_CTRL:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time) || loss_target_time_count > 3 || in_landing)
		{
			if (in_landing)
			{
				ROS_INFO("[px4ctrl] Need AUTO_HOVER(L2) Because of Landing!");
			}
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}
		break;
	}
	default:
		break;
	}

	// STEP2: arm/disarm the drone
	if (state == AUTO_HOVER)
	{
		if (param.takeoff_land.enable_auto_arm)
		{
			if (!in_landing)
			{
				if (rc_data.toggle_reboot)
				{
					if (state_data.current_state.armed)
					{
						// ROS_ERROR("[px4ctrl] Reject arm! The drone has already armed!");
					}
					else
					{
						toggle_arm_disarm(true); // 解锁
					}
				} // Try to arm.
				else
				{
					if (!state_data.current_state.armed)
					{
						// ROS_ERROR("[px4ctrl] Reject disarm! The drone has already disarmed!");
					}
					else
					{
						toggle_arm_disarm(false); // 锁
					}
				} // Try to disarm.
			}
			else if (u.thrust <= 0.01)
			{
				if (state_data.current_state.armed)
				{
					toggle_arm_disarm(false);
				}
			}
		}
	}

	// STEP3: estimate thrust model
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		// controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
		controller.estimateThrustModel(imu_data.a, param);
	}

	// STEP4: solve and update new control commands
	debug_msg = controller.calculateControl(des, odom_data, imu_data, u, static_cast<int>(state), in_landing);
	debug_msg.header.stamp = now_time;
	debug_pub.publish(debug_msg);
	if (in_landing)
	{
		u.q = imu_data.q;
	}
	// STEP5: publish control commands to mavros
	publish_acceleration_ctrl(u, now_time);

	// STEP6: Detect if the drone has landed
	land_detector(state, des, odom_data);
	// cout << takeoff_land.landed << endl;
	//  fflush(stdout);

	// STEP7: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	// rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;

	// STEP8: publish the current state of state machine
	publish_state();
	// cout << loss_target_time_count << endl;
}

void PX4CtrlFSM::loss_target_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	if (msg->data[4] == 0)
	{
		loss_target_time_count = 0;
	}
	else
	{
		if (loss_target_time_count < 15)
		{
			loss_target_time_count++;
		}
	}
}

void PX4CtrlFSM::landing_callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data)
	{
		in_landing = true;
	}
	else
	{
		in_landing = false;
	}
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && state == State_t::AUTO_HOVER)
	{
		takeoff_land.landed = false; // Always holds
	}
	if (state == State_t::AUTO_HOVER && des.p(2) > 0.1)
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;
	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.25; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		   // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 4;			   // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static ros::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) // Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}
		is_last_C12_satisfy = C12_satisfy;
	}
	// cout << takeoff_land.landed << endl;
}

Desired_State_t PX4CtrlFSM::get_hover_des() // 先得到hover_pose,再得到hover_des
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;
	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;
	return des;
}

void PX4CtrlFSM::set_hov_with_odom() // 得到hov_pose 用惯性系的位置和偏航角来确定悬停的位置和偏航，因为是悬停，所以不涉及速度、加速度以及俯仰角、滚转角等
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc() // 得到hov_pose 用遥控器来决定无人机的悬停位置和偏航角
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;
	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? -1 : 1);	// 通道1决定俯仰角？
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);		// 通道0决定滚转角？
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? -1 : 1); // 通道2是油门？
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);		// 通道3是偏航？
	//cout << hover_pose(3) << endl;
	if (hover_pose(2) < -0.35) // 不能再往下面走了，已经在地面上了
		hover_pose(2) = -0.35;
	while (hover_pose(3) > M_PI)
		hover_pose(3) -= 2.0 * M_PI;
	while (hover_pose(3) < -M_PI)
		hover_pose(3) += 2.0 * M_PI;
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_acceleration_ctrl(const Controller_Output_t &u, const ros::Time &stamp) // 发送姿态和力矩指令
{
	mavros_msgs::AttitudeTarget msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");
	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();
	msg.thrust = u.thrust;
	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	traj_start_trigger_pub.publish(msg);
}

void PX4CtrlFSM::publish_state()
{
	std_msgs::Int32 state_msg;
	state_msg.data = static_cast<int>(state); // 将枚举转换为整数
	state_pub.publish(state_msg);
	// state_pub.publish(state);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
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

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}
