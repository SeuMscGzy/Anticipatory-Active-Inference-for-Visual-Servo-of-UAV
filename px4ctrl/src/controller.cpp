#include "controller.h"

LinearControl::LinearControl(Parameter_t &param)
    : param_(param)
{
  resetThrustMapping();
}

quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom,
    const Imu_Data_t &imu, Controller_Output_t &u,
    int state_count, bool in_landing)
{
  // Initialize on first call
  if (first_time_in_function)
  {
    initial_odom_position = odom.p;
    first_time_in_function = false;
  }

  // Update flight state and calculate outputs------得到飞行状态
  updateFlightState(des, odom, state_count, in_landing);

  // Handle different control modes------得到des_acc
  if (state_count == 1)
  {
    handleManualControl(u);
  }
  else if (state_count == 2)
  {
    handleHoverControl(des, odom);
  }
  else
  {
    handleCommandControl(des);
  }

  calculateThrustAndAcc(u);//得到推力（对应发送姿态的模式）和期望加速度（对应发送线加速度的模式）
  calculateAttitude(des.yaw, odom, u, state_count);//得到期望姿态（对应发送姿态的模式）

  // Update debug message
  updateDebugMsg(des, u);
  return debug_msg_;
}

void LinearControl::updateFlightState(
    const Desired_State_t &des, const Odom_Data_t &odom,
    int state_count, bool in_landing)
{
  altitude_diff_ = odom.p[2] - initial_odom_position[2];
  switch (flight_state)
  {
  case FlightState::GROUND:
    handleGroundState(des, odom, state_count, in_landing);
    break;
  case FlightState::SLOW_START:
    handleSlowStartState(des, odom, state_count);
    break;
  case FlightState::FLYING:
    handleFlyingState(des, odom, in_landing);
    break;
  case FlightState::LANDING:
    handleLandingState(in_landing);
    break;
  }
}

void LinearControl::handleGroundState(
    const Desired_State_t &des, const Odom_Data_t &odom,
    int state_count, bool in_landing)
{

  slow_start_step_ = 0;
  land_step_ = 0;
  if (state_count == 2 && !in_landing && des.p[2] > 0.2)
  {
    if (satisfy_times_takeoff_ < 10)
    {
      satisfy_times_takeoff_++;
    }
    else
    {
      ROS_INFO("Slow speed up and ready to Takeoff!");
      flight_state = FlightState::SLOW_START;
      satisfy_times_on_ground_ = 0;
    }
  }
}
void LinearControl::handleSlowStartState(
    const Desired_State_t &des, const Odom_Data_t &odom,
    int state_count)
{

  satisfy_times_on_ground_ = 0;
  if (slow_start_step_ == kSlowStartSteps)
  {
    if (altitude_diff_ > kGroundAltThreshold && state_count == 2)
    {
      ROS_INFO("Takeoff completed, already in the air!");
      flight_state = FlightState::FLYING;
    }
    else
    {
      if (state_count == 1)
      {
        ROS_INFO("Takeoff failed due to manual control");
        flight_state = FlightState::GROUND;
        satisfy_times_takeoff_ = 0;
      }
      else if (des.p[2] < -0.2)
      {
        ROS_INFO("Takeoff canceled!");
        flight_state = FlightState::GROUND;
        satisfy_times_takeoff_ = 0;
      }
      else
      {
        ROS_INFO("Takeoff failed! Check thrust parameters!");
        flight_state = FlightState::GROUND;
      }
    }
  }
  else if (des.p[2] <= -0.2 && altitude_diff_ < kGroundAltThreshold &&
           state_count == 2)
  {
    ROS_INFO("Takeoff canceled!");
    flight_state = FlightState::GROUND;
    satisfy_times_takeoff_ = 0;
  }
}

void LinearControl::handleFlyingState(
    const Desired_State_t &des, const Odom_Data_t &odom, bool in_landing)
{
  if (in_landing)
  {
    ROS_INFO("Start Landing!");
    flight_state = FlightState::LANDING;
  }
  else if (des.p[2] <= -2 && odom.v[2] > -0.5)
  {
    satisfy_times_on_ground_++;
    if (satisfy_times_on_ground_ >= 10)
    {
      ROS_INFO("Landing over during flying state!");
      flight_state = FlightState::GROUND;
      satisfy_times_takeoff_ = 0;
    }
  }
  else
  {
    satisfy_times_on_ground_ = 0;
  }
}

void LinearControl::handleLandingState(bool in_landing)
{
  satisfy_times_on_ground_ = 0;
  if (in_landing && land_step_ == kLandingSteps)
  {
    ROS_INFO("Landing completed, already on the ground!");
    satisfy_times_takeoff_ = 0;
  }
}

void LinearControl::calculateThrustAndAcc(
    Controller_Output_t &u)
{
  double desired_thrust = kMinThrust;
  switch (flight_state)
  {
  case FlightState::GROUND:
    desired_thrust = kMinThrust;
    velocity_controller_x_.init();
    velocity_controller_y_.init();
    velocity_controller_z_.init();
    break;
  case FlightState::SLOW_START:
    desired_thrust = computeDesiredCollectiveThrustSignal() *
                     slow_start_step_ * kSlowStartFactor;
    slow_start_step_ = min(slow_start_step_ + 1, kSlowStartSteps);
    velocity_controller_x_.init();
    velocity_controller_y_.init();
    velocity_controller_z_.init();
    break;
  case FlightState::FLYING:
    desired_thrust = computeDesiredCollectiveThrustSignal();
    u.acc = des_acc - Vector3d(0, 0, param_.gra);
    break;
  case FlightState::LANDING:
    desired_thrust = (land_step_ == 0) ? last_thrust_ : last_thrust_ * 0.95;
    land_step_ = min(land_step_ + 1, kLandingSteps);
    if (land_step_ == kLandingSteps)
    {
      desired_thrust = 0.01;
    }
    break;
  }
  u.thrust = clamp(desired_thrust, kMinThrust, kMaxThrust);
  last_thrust_ = u.thrust;
}

void LinearControl::calculateAttitude(
    double yaw, const Odom_Data_t &odom, Controller_Output_t &u, int state_count)
{
  const double yaw_odom = fromQuaternion2yaw(odom.q);
  const double sin_yaw = sin(yaw_odom);
  const double cos_yaw = cos(yaw_odom);
  const double roll = (des_acc(0) * sin_yaw - des_acc(1) * cos_yaw) / param_.gra;
  const double pitch = (des_acc(0) * cos_yaw + des_acc(1) * sin_yaw) / param_.gra;
  u.q = AngleAxisd(yaw, Vector3d::UnitZ()) *
        AngleAxisd(pitch, Vector3d::UnitY()) *
        AngleAxisd(roll, Vector3d::UnitX());
  u.yaw = yaw;
  if (flight_state == FlightState::FLYING && state_count == 2)
  {
    timed_thrust_.emplace(ros::Time::now(), u.thrust);
    while (timed_thrust_.size() > 100)
    {
      timed_thrust_.pop();
    }
  }
}

void LinearControl::handleManualControl(Controller_Output_t &u)
{
  velocity_controller_x_.init();
  velocity_controller_y_.init();
  velocity_controller_z_.init();
  u.acc = Vector3d(0, 0, -1);
}

void LinearControl::handleHoverControl(
    const Desired_State_t &des, const Odom_Data_t &odom)
{
  if (pos_vel_control_counter_ % 2 == 0)
  {
    desire_v_x = kPositionGainHorizontal *
                 (des.p[0] - (odom.p[0] - initial_odom_position[0]));
    desire_v_y = kPositionGainHorizontal *
                 (des.p[1] - (odom.p[1] - initial_odom_position[1]));
  }
  des_acc[0] = velocity_controller_x_.update(desire_v_x - odom.v[0]);
  des_acc[1] = velocity_controller_y_.update(desire_v_y - odom.v[1]);
  des_acc[2] = velocity_controller_z_.update(des.p[2] - odom.v[2]);
  pos_vel_control_counter_++;
  des_acc += Vector3d(0, 0, param_.gra);
}

void LinearControl::handleCommandControl(
    const Desired_State_t &des)
{
  velocity_controller_x_.init();
  velocity_controller_y_.init();
  velocity_controller_z_.init();
  des_acc = des.a + Vector3d(0, 0, param_.gra);
}

double LinearControl::fromQuaternion2yaw(const Quaterniond &q)
{
  return atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                    1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}

double LinearControl::computeDesiredCollectiveThrustSignal()
{

  if (abs(thr2acc) < 1e-6)
  {
    ROS_ERROR_THROTTLE(1.0, "Invalid thr2acc value: %f", thr2acc);
    return kMinThrust;
  }
  return des_acc(2) / thr2acc;
}

bool LinearControl::estimateThrustModel(const Vector3d &est_a, const Parameter_t &param)
{
  const ros::Time t_now = ros::Time::now();
  while (!timed_thrust_.empty())
  {
    auto t_t = timed_thrust_.front();
    const double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045)
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035)
    {
      return false;
    }
    const double thr = t_t.second;
    timed_thrust_.pop();
    if (thr > 0.01 && flight_state == FlightState::FLYING)
    {
      const double gamma = 1 / (kRho2 + thr * P * thr);
      const double K = gamma * P * thr;
      thr2acc += K * (est_a(2) - thr * thr2acc);
      P = (1 - K * thr) * P / kRho2;
      ROS_DEBUG("Updated thr2acc: %f", thr2acc);
      return true;
    }
  }
  return false;
}
void LinearControl::resetThrustMapping()
{
  thr2acc = param_.gra / param_.thr_map.hover_percentage;
  P = 1e6;
}

void LinearControl::updateDebugMsg(
    const Desired_State_t &des, const Controller_Output_t &u)
{
  // used for debug
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
}