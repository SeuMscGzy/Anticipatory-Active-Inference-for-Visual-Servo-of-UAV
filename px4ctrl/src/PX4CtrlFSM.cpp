#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_)
    : param(param_), nh("~"), loss_target_time_count(10),
      controller(controller_)
{
  state = MANUAL_CTRL;
}

/*
        Finite State Machine

              MANUAL_CTRL
                 ^   |
                 |   |
                 |   |
                 |   |
                 |   |
                 |   |
                 |   |
                 |	 |
                 |   |
                 |   v
                CMD_CTRL

*/

void PX4CtrlFSM::process()
{
  ros::Time now_time = ros::Time::now();
  Controller_Output_t u;
  Desired_State_t des; // des代表了当前的无人机位姿，利用初始化函数读取里程计信息来给其赋值
  des = get_cmd_des(); // 自主控制需要改的函数
  // STEP1: state machine runs
  switch (state)
  {
  case MANUAL_CTRL:
  {
    if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
    {
      if (!odom_is_received(now_time))
      {
        ROS_ERROR("[px4ctrl] Reject CMD_CTRL(L2). No odom!");
        break;
      }
      if (cmd_is_received(now_time) && loss_target_time_count == 0)
      {
        toggle_offboard_mode(true);
        if (state_data.current_state.mode == "OFFBOARD")
        {
          state = CMD_CTRL;
          ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> CMD_CTRL(L2)\033[32m");
        }
      }
      else // 悬停模式可以用遥控器来悬停，如果遥控器不操作的话就是保持最开始进入悬停时的位置和偏航角
      {
        ROS_INFO("no ctrl cmd, waiting for cmd or loss target");
      }
    }
    break;
  }
  case CMD_CTRL:
  {
    if (!rc_data.is_hover_mode || !odom_is_received(now_time) || !cmd_is_received(now_time) ||
        loss_target_time_count > 3)
    {
      state = MANUAL_CTRL;
      toggle_offboard_mode(false);
      ROS_WARN("[px4ctrl] From CMD_CTRL(L2) to MANUAL_CTRL(L1)!");
    }
    break;
  }
  default:
    break;
  }

  // STEP2: solve and update new control commands
  debug_msg = controller.calculateControl(des, u);
  debug_msg.header.stamp = now_time;
  debug_pub.publish(debug_msg);
  // STEP5: publish control commands to mavros
  publish_acceleration_ctrl(u, now_time);

  // STEP6: Clear flags beyound their lifetime
  rc_data.enter_hover_mode = false;
}

void PX4CtrlFSM::loss_target_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (msg->data[4] == 0)
  {
    loss_target_time_count = 0;
  }
  else
  {
    if (loss_target_time_count < 10)
    {
      loss_target_time_count++;
    }
  }
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

void PX4CtrlFSM::publish_acceleration_ctrl(
    const Controller_Output_t &u, const ros::Time &stamp) // 发送姿态和力矩指令
{
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

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
  mavros_msgs::SetMode offb_set_mode;
  if (on_off)
  {
    state_data.state_before_offboard = state_data.current_state;
    if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
      state_data.state_before_offboard.mode = "MANUAL";

    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (!(set_FCU_mode_srv.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent))
    {
      ROS_ERROR("Enter OFFBOARD rejected by PX4!");
      return false;
    }
  }
  else
  {
    offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
    if (!(set_FCU_mode_srv.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent))
    {
      ROS_ERROR("Exit OFFBOARD rejected by PX4!");
      return false;
    }
  }

  return true;
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
