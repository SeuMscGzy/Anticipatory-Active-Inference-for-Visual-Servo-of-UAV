#include "controller.h"

LinearControl::LinearControl(Parameter_t &param)
    : param_(param)
{
  resetThrustMapping();
}

quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom,
    const Imu_Data_t &imu, Controller_Output_t &u,
    int state_count)
{
  // Initialize on first call
  if (first_time_in_function)
  {
    initial_odom_position = odom.p;
    first_time_in_function = false;
  }

  // Handle different control modes------得到des_acc
  if (state_count == 1)
  {
    handleManualControl();
  }
  else if (state_count == 2)
  {
    // Update flight state and calculate outputs------得到飞行状态
    updateFlightState(des, odom);
    handleHoverControl(des, odom);
  }
  else
  {
    handleCommandControl(des);
  }

  calculateAttitude(des.yaw, odom, u); // 得到期望姿态（对应发送姿态的模式）
  calculateThrust(u);                  // 得到推力（对应发送姿态的模式）和期望加速度（对应发送线加速度的模式）

  // Update debug message
  updateDebugMsg(des, u);
  return debug_msg_;
}

void LinearControl::updateFlightState(
    const Desired_State_t &des, const Odom_Data_t &odom)
{
  altitude_diff_ = odom.p[2] - initial_odom_position[2];
  switch (flight_state)
  {
  case FlightState::GROUND:
    if (des.p[2] > 0.2)
    {
      if (satisfy_times_takeoff_ < 10)
      {
        satisfy_times_takeoff_++;
      }
      else
      {
        ROS_INFO("Slow speed up and ready to Takeoff!");
        flight_state = FlightState::SLOW_START;
        slow_start_step_ = 0;
        satisfy_times_on_ground_ = 0;
      }
    }
    else
    {
      satisfy_times_takeoff_ = 0;
    }
    break;
  case FlightState::SLOW_START:
    if (slow_start_step_ == kSlowStartSteps)
    {
      ROS_INFO("Takeoff completed, already in the air!");
      flight_state = FlightState::FLYING;
      slow_start_step_ = 0;
    }
    else
    {
      slow_start_step_++;
      if (des.p[2] < -0.2)
      {
        ROS_WARN("Takeoff cancelled during slow start!");
        flight_state = FlightState::GROUND;
        satisfy_times_takeoff_ = 0;
        slow_start_step_ = 0;
      }
    }
    break;
  case FlightState::FLYING:
    if (des.p[2] <= -2 && odom.v[2] > -0.5 && abs(odom.v[0] < 0.2) && abs(odom.v[1] < 0.2))
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
    break;
  }
}

void LinearControl::handleManualControl()
{
  velocity_controller_x_.init();
  velocity_controller_y_.init();
  velocity_controller_z_.init();
  des_acc = Vector3d(0, 0, param_.gra - 1);
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
  des_acc[2] = velocity_controller_z_.update(des.p[2] - odom.v[2]); // 油门推杆给的是速度期望
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

void LinearControl::calculateAttitude(double yaw, const Odom_Data_t &odom, Controller_Output_t &u)
{
  const double yaw_odom = fromQuaternion2yaw(odom.q);
  const double sin_yaw = sin(yaw_odom);
  const double cos_yaw = cos(yaw_odom);
  const double roll = (des_acc(0) * sin_yaw - des_acc(1) * cos_yaw) / param_.gra;
  const double pitch = (des_acc(0) * cos_yaw + des_acc(1) * sin_yaw) / param_.gra;
  u.q = AngleAxisd(yaw, Vector3d::UnitZ()) *
        AngleAxisd(pitch, Vector3d::UnitY()) *
        AngleAxisd(roll, Vector3d::UnitX());

  // SE(3)
  //  3) 期望姿态
  Vector3d b3d = des_acc.normalized();
  Vector3d a_psi{cos(yaw), sin(yaw), 0};

  Vector3d b2d = b3d.cross(a_psi).normalized();
  Vector3d b1d = b2d.cross(b3d);

  Matrix3d R_d;
  R_d.col(0) = b1d;
  R_d.col(1) = b2d;
  R_d.col(2) = b3d;
  Quaterniond q_SE3(R_d);
  R_w2i = odom.q.toRotationMatrix(); // 传感器到世界坐标系的旋转矩阵
  b3_in_world = R_w2i.col(2);
  des_acc_SE3 = des_acc.dot(b3_in_world);
  double thrust_z = des_acc_SE3 / thr2acc * b3_in_world[2];

  // 测试两种姿态解算方法的一致性
  //  1. 相对四元数
  Quaterniond q_err = q_SE3 * u.q.conjugate();

  // 2. 旋转误差角度
  double angle_err = 2.0 * std::acos(q_err.w()); // rad

  // 3. 误差轴
  Vector3d axis_err;
  double s = std::sqrt(1.0 - q_err.w() * q_err.w());
  if (s < 1e-6)
  {
    axis_err = Vector3d::UnitX(); // 任意轴
  }
  else
  {
    axis_err = q_err.vec() / s;
  }

  // 4. 打印
  cout << "Attitude error (rad): " << angle_err << std::endl;
  cout << "Error axis: ["
       << axis_err.transpose() << "]" << std::endl;

  if (flight_state == FlightState::FLYING)
  {
    timed_thrust_.emplace(ros::Time::now(), thrust_z);
    while (timed_thrust_.size() > 100)
    {
      timed_thrust_.pop();
    }
  }
}

void LinearControl::calculateThrust(Controller_Output_t &u)
{
  double desired_thrust = kMinThrust;
  switch (flight_state)
  {
  case FlightState::GROUND:
    desired_thrust = kMinThrust;
    break;
  case FlightState::SLOW_START:
    desired_thrust = computeDesiredCollectiveThrustSignal() *
                     slow_start_step_ * kSlowStartFactor;
    break;
  case FlightState::FLYING:
    desired_thrust = computeDesiredCollectiveThrustSignal();
    break;
  }
  u.thrust = clamp(desired_thrust, kMinThrust, kMaxThrust);
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