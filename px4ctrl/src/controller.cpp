#include "controller.h"

LinearControl::LinearControl(Parameter_t &param)
    : param_(param)
{
}

quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, Controller_Output_t &u)
{
  handleCommandControl(u, des); // 得到推力（对应发送姿态的模式）和期望加速度（对应发送线加速度的模式）
  // Update debug message
  updateDebugMsg(des, u);
  return debug_msg_;
}

void LinearControl::handleCommandControl(Controller_Output_t &u, const Desired_State_t &des)
{
  u.acc = des.a;
}

void LinearControl::updateDebugMsg(
    const Desired_State_t &des, const Controller_Output_t &u)
{
  // used for debug
  debug_msg_.des_v_x = 0;
  debug_msg_.des_v_y = 0;
  debug_msg_.des_v_z = 0;

  debug_msg_.des_a_x = u.acc(0);
  debug_msg_.des_a_y = u.acc(1);
  debug_msg_.des_a_z = u.acc(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
}