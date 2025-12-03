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
Eigen::Vector3d last_des_acc(0.0, 0.0, 0.0);
Eigen::Vector3d des_vel(0.0, 0.0, 0.0);
Eigen::Vector3d Kp, Kv;
double last_pos_vel_time = 0;

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
    if (abs(u_pid) > 2)
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
  if (state_count == 1)
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    des_acc.setZero();
    u.acc[0] = 0;
    u.acc[1] = 0;
    u.acc[2] = -1;
  }
  else if (state_count == 2)
  {
    if (pos_vel_control_counter % 2 == 0)
    {
      // cout << "pos_time: " << 1000 * (ros::Time::now().toSec() - last_pos_vel_time) << endl;
      // last_pos_vel_time = ros::Time::now().toSec();
      desire_v_x = position_controller_gain_horizontal * (des.p[0] - odom.p[0]);
      desire_v_y = position_controller_gain_horizontal * (des.p[1] - odom.p[1]);
      desire_v_z = position_controller_gain_vertical * (des.p[2] - odom.p[2]);
      pos_vel_control_counter = 0;
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
      last_des_acc = des_acc;
    }
    else
    {
      des_acc = last_des_acc;
    }
  }
  else
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    des_acc = des.a;
  }
  // cout << des_acc[2] << " " << pos_vel_control_counter << endl;
  pos_vel_control_counter++;
  des_acc += Eigen::Vector3d(0, 0, param_.gra);
  // 避免 0 向量
  Eigen::Vector3d e3(0.0, 0.0, 1.0);
  if (des_acc.norm() < 1e-6)
  {
    des_acc = param_.gra * e3; // 退化到悬停
  }
  Eigen::Vector3d b3 = des_acc.normalized();

  // 2) 期望 yaw -> 参考 b1c
  double psi = des.yaw; // 期望航向（世界系）
  Eigen::Vector3d b1c(std::cos(psi), std::sin(psi), 0.0);

  // 3) 用叉乘构造 b1, b2
  Eigen::Vector3d b2 = b3.cross(b1c);
  if (b2.norm() < 1e-6)
  {
    // 说明 b3 和 b1c 几乎平行，随便取一个正交方向
    b2 = Eigen::Vector3d(0.0, 0.0, 1.0).cross(b3);
  }
  b2.normalize();

  Eigen::Vector3d b1 = b2.cross(b3);

  // 4) 得到期望旋转矩阵（世界->机体系）
  Eigen::Matrix3d R_des;
  R_des.col(0) = b1;
  R_des.col(1) = b2;
  R_des.col(2) = b3;

  // 5) 转成四元数
  Eigen::Quaterniond q_des(R_des);
  q_des.normalize();

  // 6) 和你原来的坐标系补偿拼在一起
  u.q = imu.q * odom.q.inverse() * q_des;

  Eigen::Vector3d b3_curr = odom.q.toRotationMatrix().col(2); // 机体 z 轴在世界系中的方向
  u.thrust = computeDesiredCollectiveThrustSignal(des_acc.dot(b3_curr));
  double cos_tilt = b3_curr.dot(e3);
  if (state_count == 1)
  {
    u.thrust = 0.01;
  }
  if ((last_state_count == 1 && state_count == 2) || in_landing_)
  {
    enter_count = 0;
  }
  if ((state_count != 3 && des.p[2] < -0.2 && odom.p[2] < 0.1 && des_acc[2] < param_.gra) || enter_count < 200) // 在地上或快降落到地上且推杆在底部或中部且为auto_hover模式
  {
    in_the_slow_thrust = true;
    enter_count++;
    if (enter_count > 200)
    {
      enter_count = 200;
    }
    last_thrust *= 0.995;
    if (last_thrust < 0.01)
    {
      last_thrust = 0.01;
    }
    u.thrust = last_thrust;
  }
  else
  {
    in_the_slow_thrust = false;
  }
  if (last_in_the_slow_thrust == true && in_the_slow_thrust == false && state_count != 3 && !in_landing_) // 缓慢加速起飞
  {
    takeoff_count = 0;
  }
  if (takeoff_count < 500)
  {
    takeoff_count++;
    u.thrust = u.thrust * takeoff_count * 0.01 * 0.2; // 从百分之1逐渐加速到百分之百的推力
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
  }
  last_in_the_slow_thrust = in_the_slow_thrust;
  last_state_count = state_count;
  last_thrust = u.thrust;
  if (takeoff_count >= 500)
  {
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), cos_tilt * u.thrust));
  }
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
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
    const double &thrust)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = thrust / thr2acc_;

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
    if (thr > 0.01 && takeoff_count >= 500)
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
