#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_)
    : param(param_), nh("~"), loss_target_time_count(10),
      controller(controller_) {
  state = MANUAL_CTRL;
  hover_pose.setZero();
}

/*
        Finite State Machine

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

void PX4CtrlFSM::process() {
  ros::Time now_time = ros::Time::now();
  Controller_Output_t u;
  u.thrust = 0;
  Desired_State_t des(
      odom_data); // des代表了当前的无人机位姿，利用初始化函数读取里程计信息来给其赋值

  // STEP1: state machine runs
  switch (state) {
  case MANUAL_CTRL: {
    if (rc_data
            .enter_hover_mode) // Try to jump to AUTO_HOVER
                               // 这个变量是怎么赋值的？根据通道的信息来确定吗？
    {
      if (!odom_is_received(now_time)) {
        ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
        break;
      }
      if (odom_data.v.norm() > 3.0) {
        ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which "
                  "seems that the locolization module goes wrong!",
                  odom_data.v.norm());
        break;
      }

      state = AUTO_HOVER;
      controller.resetThrustMapping();
      set_hov_with_rc(); // 使用里程计当前的信息进行悬停
      toggle_offboard_mode(true);
      ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
    }
    break;
  }
  case AUTO_HOVER: {
    if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
      state = MANUAL_CTRL;
      toggle_offboard_mode(false);
      ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
    } else if (rc_data.is_command_mode && cmd_is_received(now_time) &&
               loss_target_time_count == 0 && !in_landing) {
      if (state_data.current_state.mode == "OFFBOARD") {
        state = CMD_CTRL;
        des = get_cmd_des(); // 自主控制需要改的函数
        ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
      }
    } else // 悬停模式可以用遥控器来悬停，如果遥控器不操作的话就是保持最开始进入悬停时的位置和偏航角
    {
      set_hov_with_rc();
      des = get_hover_des();
      if (rc_data.enter_command_mode) {
        ROS_INFO("no ctrl cmd, waiting for cmd");
      }
    }
    break;
  }

  case CMD_CTRL: {
    if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
      state = MANUAL_CTRL;
      toggle_offboard_mode(false);
      ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
    } else if (!rc_data.is_command_mode || !cmd_is_received(now_time) ||
               loss_target_time_count > 3 || in_landing) {
      if (in_landing) {
        ROS_INFO("[px4ctrl] Need AUTO_HOVER(L2) Because of Landing!");
      }
      state = AUTO_HOVER;
      set_hov_with_odom();
      des = get_hover_des();
      ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
    } else {
      des = get_cmd_des();
    }
    break;
  }
  default:
    break;
  }

  // STEP2: arm/disarm the drone
  if (state == AUTO_HOVER) {
    if (param.takeoff_land.enable_auto_arm) {
      if (!in_landing) {
        if (rc_data.toggle_reboot) {
          if (state_data.current_state.armed) {
            // ROS_ERROR("[px4ctrl] Reject arm! The drone has already armed!");
          } else {
            toggle_arm_disarm(true); // 解锁
          }
        } // Try to arm.
        else {
          if (!state_data.current_state.armed) {
            // ROS_ERROR("[px4ctrl] Reject disarm! The drone has already
            // disarmed!");
          } else {
            toggle_arm_disarm(false); // 锁
          }
        } // Try to disarm.
      } else if (u.thrust <= 0.01) {
        if (state_data.current_state.armed) {
          toggle_arm_disarm(false);
        }
      }
    }
  }

  // STEP3: estimate thrust model
  if (state == AUTO_HOVER) {
    // controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
    controller.estimateThrustModel(imu_data.a, param);
  }

  // STEP4: solve and update new control commands
  debug_msg = controller.calculateControl(des, odom_data, imu_data, u,
                                          static_cast<int>(state), in_landing);
  debug_msg.header.stamp = now_time;
  debug_pub.publish(debug_msg);
  if (in_landing ||
      controller.flight_state == LinearControl::FlightState::LANDING) {
    u.q = imu_data.q;
  }
  // STEP5: publish control commands to mavros
  publish_acceleration_ctrl(u, now_time);

  // STEP7: Clear flags beyound their lifetime
  rc_data.enter_hover_mode = false;
  rc_data.enter_command_mode = false;
}

void PX4CtrlFSM::loss_target_callback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  if (msg->data[4] == 0) {
    loss_target_time_count = 0;
  } else {
    if (loss_target_time_count < 10) {
      loss_target_time_count++;
    }
  }
}

void PX4CtrlFSM::landing_callback(const std_msgs::Bool::ConstPtr &msg) {
  in_landing = msg->data;
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

Desired_State_t PX4CtrlFSM::get_cmd_des() {
  Desired_State_t des;
  des.p = cmd_data.p;
  des.v = cmd_data.v;
  des.a = cmd_data.a;
  des.j = cmd_data.j;
  des.yaw = cmd_data.yaw;
  des.yaw_rate = cmd_data.yaw_rate;
  return des;
}

void PX4CtrlFSM::
    set_hov_with_odom() // 得到hov_pose
                        // 用惯性系的位置和偏航角来确定悬停的位置和偏航，因为是悬停，所以不涉及速度、加速度以及俯仰角、滚转角等
{
  hover_pose.head<3>() = odom_data.p;
  hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

  last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc() // 得到hov_pose
                                   // 用遥控器来决定无人机的悬停位置和偏航角
{
  ros::Time now = ros::Time::now();
  double delta_t = (now - last_set_hover_pose_time).toSec();
  last_set_hover_pose_time = now;
  hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t *
                   (param.rc_reverse.pitch ? -1 : 1); // 通道1决定俯仰角？
  hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t *
                   (param.rc_reverse.roll ? 1 : -1); // 通道0决定滚转角？
  hover_pose(2) = 2.1 * rc_data.ch[2];               // 通道2是油门？
  //cout << " rc_data.ch[2]: " << rc_data.ch[2] << endl;
  hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t *
                   (param.rc_reverse.yaw ? 1 : -1); // 通道3是偏航？

  if (hover_pose(2) < -2) // 最大下降速度
    hover_pose(2) = -2;
  if (hover_pose(2) > 2) // 最大上升速度
    hover_pose(2) = 2;
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time) {
  return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time) {
  return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time) {
  return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time) {
  return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time) {
  return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

void PX4CtrlFSM::publish_acceleration_ctrl(
    const Controller_Output_t &u, const ros::Time &stamp) // 发送姿态和力矩指令
{
  if (state == AUTO_HOVER) {
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
    ctrl_FCU_pub_att.publish(msg);
  } else {
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    // 设置你想要的加速度值
    msg.acceleration_or_force.x = u.acc[0]; // X轴加速度
    msg.acceleration_or_force.y = u.acc[1]; // Y轴加速度
    msg.acceleration_or_force.z = u.acc[2];
    msg.yaw = u.yaw; // 偏航角，例如，1.57弧度约等于90度
    ctrl_FCU_pub_acc.publish(msg);
  }
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off) {
  mavros_msgs::SetMode offb_set_mode;

  if (on_off) {
    state_data.state_before_offboard = state_data.current_state;
    if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
      state_data.state_before_offboard.mode = "MANUAL";

    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (!(set_FCU_mode_srv.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)) {
      ROS_ERROR("Enter OFFBOARD rejected by PX4!");
      return false;
    }
  } else {
    offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
    if (!(set_FCU_mode_srv.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)) {
      ROS_ERROR("Exit OFFBOARD rejected by PX4!");
      return false;
    }
  }

  return true;

  // if (param.print_dbg)
  // 	printf("offb_set_mode mode_sent=%d(uint8_t)\n",
  // offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm) {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = arm;
  if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success)) {
    if (arm)
      ROS_ERROR("ARM rejected by PX4!");
    else
      ROS_ERROR("DISARM rejected by PX4!");

    return false;
  }

  return true;
}
