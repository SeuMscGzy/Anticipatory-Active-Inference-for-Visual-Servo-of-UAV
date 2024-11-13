#include "controller.h"
#include <Eigen/Core>
using namespace std;

int pos_vel_control_counter = 0; // 位置速度控制器的计数器
const double dt = 0.02;          // 速度控制循环的时间间隔，50Hz
double desire_v_x = 0;
double desire_v_y = 0;
double desire_v_z = 0;
const double position_controller_gain_horizontal = 0.9; // 水平位置比环例控制增益
const double position_controller_gain_vertical = 1;     // 水平位置比环例控制增益
Eigen::Vector3d des_acc(0.0, 0.0, 0.0);

class PIDController
{
public:
  double Kp, Ki, Kd;
  double integral_error;

  PIDController(double Kp_, double Ki_, double Kd_) : integral_error(0)
  {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  }

  double update(double error)
  {
    integral_error += error * dt;
    if (abs(integral_error) > 1)
    {
      integral_error = 1 * integral_error / abs(integral_error);
    }
    double u_pid = Kp * error + Ki * integral_error;
    if (abs(u_pid > 2))
    {
      u_pid = 2 * u_pid / abs(u_pid);
    }
    return u_pid;
  }
  void init()
  {
    integral_error = 0;
  }
};
PIDController velocity_controller_x(2, 0.4, 0); // x轴速度环PID参数
PIDController velocity_controller_y(2, 0.4, 0); // y轴速度环PID参数
PIDController velocity_controller_z(4, 2, 0);   // z轴速度环PID参数

void LinearControl::updateFlightState(const Desired_State_t &des, const Odom_Data_t &odom, int state_count, bool in_landing_)
{
  // 更新飞行状态
  switch (flight_state)
  {
  case GROUND:
  {
    slow_start_step = 0;
    land_step = 0;
    if (state_count == 2 && !in_landing_ && des.p[2] > 0.2)
    {
      flight_state = SLOW_START;
    }
    break;
  }
  case SLOW_START:
  {
    if (slow_start_step == 100)
    {
      flight_state = FLYING;
    }
    else if (des.p[2] < -0.1)
    {
      flight_state = GROUND;
    }
    break;
  }
  case FLYING:
  {
    if (in_landing_ || (des.p[2] <= -0.1 && odom.p[2] <= LANDING_ALTITUDE_THRESHOLD && state_count == 2))
    {
      flight_state = LANDING;
    }
    break;
  }
  case LANDING:
  {
    if (odom.p[2] <= LANDING_ALTITUDE_THRESHOLD && land_step == 100)
    {
      flight_state = GROUND;
    }
    break;
  }
  default:
  {
    flight_state = GROUND;
    break;
  }
  }
}

void LinearControl::calculateThrust(Controller_Output_t &u, const Eigen::Vector3d &des_acc)
{
  double desired_thrust;
  switch (flight_state)
  {
  case GROUND:
    u.thrust = MIN_THRUST;
    // 重置PID控制器
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    break;

  case SLOW_START:
    desired_thrust = computeDesiredCollectiveThrustSignal(des_acc);
    // 平滑增加推力
    u.thrust = desired_thrust * slow_start_step * 0.01;
    slow_start_step++;
    if (slow_start_step > 100)
    {
      slow_start_step = 100;
    }
    else
    {
      velocity_controller_x.init();
      velocity_controller_y.init();
      velocity_controller_z.init();
    }
    break;

  case FLYING:
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    u.acc = des_acc;
    break;

  case LANDING:
    // 平滑减小推力
    if (land_step == 0) // 第一次进入降落状态时，记录悬停推力
    {
      u.thrust = computeDesiredCollectiveThrustSignal(param_.gra * Eigen::Vector3d(0, 0, 1));
    }
    else // 从悬停推力开始逐渐减小推力
    {
      u.thrust = last_thrust * 0.95;
    }
    land_step++;
    if (u.thrust < MIN_THRUST)
      u.thrust = MIN_THRUST;
    if (land_step > 100)
    {
      land_step = 100;
    }
    break;

  default:
    u.thrust = MIN_THRUST;
    break;
  }

  // 确保推力在最小和最大值之间
  if (u.thrust < MIN_THRUST)
    u.thrust = MIN_THRUST;
  else if (u.thrust > MAX_THRUST)
    u.thrust = MAX_THRUST;

  last_thrust = u.thrust;
}

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(const Desired_State_t &des,
                                                             const Odom_Data_t &odom,
                                                             const Imu_Data_t &imu,
                                                             Controller_Output_t &u, int state_count, bool in_landing_)
{
  if (state_count == 1) // manual control
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    u.acc[0] = 0;
    u.acc[1] = 0;
    u.acc[2] = -1;
  }
  else if (state_count == 2) // hover with rc
  {
    if (pos_vel_control_counter % 2 == 0)
    {
      desire_v_x = position_controller_gain_horizontal * (des.p[0] - odom.p[0]);
      desire_v_y = position_controller_gain_horizontal * (des.p[1] - odom.p[1]);
      desire_v_z = position_controller_gain_vertical * (des.p[2] - odom.p[2]);
      pos_vel_control_counter = 0;
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
    }
    else
    {
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
    }
    pos_vel_control_counter++;
  }
  else // cmd control
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    des_acc = des.a;
  }
  des_acc += Eigen::Vector3d(0, 0, param_.gra);
  // 更新飞行状态
  updateFlightState(des, odom, state_count, in_landing_);

  // 计算推力
  calculateThrust(u, des_acc);
  double roll, pitch;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);
  roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
  pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
  // yaw = fromQuaternion2yaw(des.q);
  Eigen::Quaterniond q_des = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = q_des; // MAP(ENU)->IMU(FLU) * IMU(FLU)->MOCAP(FLU) * MOCAP(FLU)->UAV_DES(FLU) = MAP(ENU)->UAV_DES(FLU)
  u.yaw = des.yaw;
  if (flight_state == FLYING && state_count ==2)
  {
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  }
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  // cout << "thr2acc_: " << thr2acc_ << endl;
  // cout << "takeoff_count: " << takeoff_count << endl;

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
  return debug_msg_;
}

double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    if (thr > 0.01 && flight_state == FLYING)
    {
      /***********************************/
      /* Model: est_a(2) = thr1acc_ * thr */
      /***********************************/
      double gamma = 1 / (rho2_ + thr * P_ * thr);
      double K = gamma * P_ * thr;
      thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
      P_ = (1 - K * thr) * P_ / rho2_;
    }
    // cout << thr2acc_ << endl;
    return true;
  }
  return false;
}

void LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
