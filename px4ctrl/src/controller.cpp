#include "controller.h"
#include <Eigen/Core>
using namespace std;
int satisfy_times = 0;
int pos_vel_control_counter = 0; // 位置速度控制器的计数器
const double dt = 0.02;          // 速度控制循环的时间间隔，50Hz
double desire_v_x = 0;
double desire_v_y = 0;
double desire_v_z = 0;
const double position_controller_gain_horizontal =
    0.9; // 水平位置比环例控制增益
const double position_controller_gain_vertical = 1; // 水平位置比环例控制增益
Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
bool first_time_in_function = true;
Eigen::Vector3d initial_odom_position = Eigen::Vector3d(0, 0, 0);

class PIDController {
public:
  double Kp, Ki, Kd;
  double integral_error;

  PIDController(double Kp_, double Ki_, double Kd_) : integral_error(0) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  }

  double update(double error) {
    integral_error += error * dt;
    if (abs(integral_error) > 1) {
      integral_error = 1 * integral_error / abs(integral_error);
    }
    double u_pid = Kp * error + Ki * integral_error;
    if (abs(u_pid) > 2) {
      u_pid = 2 * u_pid / abs(u_pid);
    }
    return u_pid;
  }
  void init() { integral_error = 0; }
};
PIDController velocity_controller_x(2, 0.4, 0); // x轴速度环PID参数
PIDController velocity_controller_y(2, 0.4, 0); // y轴速度环PID参数
PIDController velocity_controller_z(4, 2, 0);   // z轴速度环PID参数

void LinearControl::updateFlightState(const Desired_State_t &des,
                                      const Odom_Data_t &odom, int state_count,
                                      bool in_landing_) {
  // 第一次调用时初始化初始里程计位置
  if (first_time_in_function) {
    initial_odom_position = Eigen::Vector3d(odom.p[0], odom.p[1], odom.p[2]);
    first_time_in_function = false;
  }

  // 计算当前的高度差（相对于初始位置）
  double altitude_diff = odom.p[2] - initial_odom_position[2];

  // 输出当前状态和高度差用于调试
  cout << "Current flight state: " << flight_state << endl;
  cout << "Altitude difference: " << altitude_diff << endl;

  // 根据当前飞行状态更新状态机
  switch (flight_state) {
  case GROUND: {
    // 重置起飞和降落相关计数器
    slow_start_step = 0;
    land_step = 0;

    // 当满足起飞条件：状态采样稳定（state_count ==
    // 2），非降落中，且期望高度大于0.2时
    if (state_count == 2 && !in_landing_ && des.p[2] > 0.05) {
      if (satisfy_times < 10) {
        satisfy_times++; // 累计满足条件的次数
      } else {
        ROS_INFO("Slow speed up and ready to Takeoff!");
        flight_state = SLOW_START;
      }
    }
    break;
  }
  case SLOW_START: {
    // SLOW_START阶段：等待一定步数以便确认起飞状态
    if (slow_start_step == 200) {
      // 当达到预定步数后，判断是否成功起飞
      if (altitude_diff > GROUND_ALTITUDE_THRESHOLD && state_count == 2) {
        ROS_INFO("Takeoff completed, already in the air!");
        flight_state = FLYING;
      } else {
        // 起飞失败，根据不同情况处理
        if (state_count == 1) {
          ROS_INFO("Takeoff failed due to manual control, please switch to L2 "
                   "Auto Hover mode!");
          flight_state = GROUND;
          satisfy_times = 0;
        } else if (des.p[2] < -0.2) {
          ROS_INFO("Takeoff canceled!");
          flight_state = GROUND;
          satisfy_times = 0;
        } else {
          ROS_INFO("Takeoff failed, start landing! The thrust parameter should "
                   "be further optimized!");
          flight_state = LANDING;
        }
      }
    }
    // 在起飞启动阶段中，如果检测到取消起飞条件，则直接取消起飞
    else if (des.p[2] < -0.2 && altitude_diff < GROUND_ALTITUDE_THRESHOLD &&
             state_count == 2) {
      ROS_INFO("Takeoff canceled!");
      flight_state = GROUND;
      satisfy_times = 0;
    }
    break;
  }
  case FLYING: {
    // 在飞行中，如果收到降落指令或满足降落条件，则启动降落
    if (in_landing_ ||
        (des.p[2] <= -0.1 && altitude_diff <= LANDING_ALTITUDE_THRESHOLD &&
         state_count == 2)) {
      ROS_INFO("Start Landing!");
      flight_state = LANDING;
    }
    break;
  }
  case LANDING: {
    // 在降落过程中，判断是否已经落地（高度足够低且land_step达到要求）
    if (altitude_diff <= GROUND_ALTITUDE_THRESHOLD && land_step == 100) {
      ROS_INFO("Landing completed, already on the ground!");
      flight_state = GROUND;
      satisfy_times = 0;
    }
    break;
  }
  default: {
    // 未识别状态，安全起见恢复到地面状态
    flight_state = GROUND;
    satisfy_times = 0;
    break;
  }
  }
}

void LinearControl::calculateThrustandAcc(Controller_Output_t &u,
                                          const Eigen::Vector3d &des_acc) {
  double desired_thrust;
  switch (flight_state) {
  case GROUND: {
    u.thrust = MIN_THRUST;
    // 重置PID控制器
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    break;
  }

  case SLOW_START: {
    desired_thrust = computeDesiredCollectiveThrustSignal(des_acc);
    // 平滑增加推力
    u.thrust = desired_thrust * slow_start_step * 0.005;
    slow_start_step++;
    if (slow_start_step > 200) {
      slow_start_step = 200;
    }
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    break;
  }

  case FLYING: {
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    u.acc = des_acc -
            Eigen::Vector3d(
                0, 0,
                param_.gra); // 净加速度
                             // 而非计算推力时所需要的考虑重力加速度后的加速度。
    break;
  }

  case LANDING: {
    // 平滑减小推力
    if (land_step == 0) // 第一次进入降落状态时，记录悬停推力
    {
      u.thrust = computeDesiredCollectiveThrustSignal(param_.gra *
                                                      Eigen::Vector3d(0, 0, 1));
    } else // 从悬停推力开始逐渐减小推力
    {
      u.thrust = last_thrust * 0.95;
    }
    land_step++;
    if (u.thrust < MIN_THRUST)
      u.thrust = MIN_THRUST;
    if (land_step > 100) {
      land_step = 100;
    }
    break;
  }

  default: {
    u.thrust = MIN_THRUST;
    break;
  }
  }

  // 确保推力在最小和最大值之间
  if (u.thrust < MIN_THRUST)
    u.thrust = MIN_THRUST;
  else if (u.thrust > MAX_THRUST)
    u.thrust = MAX_THRUST;

  last_thrust = u.thrust;
}

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param) {}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu,
    Controller_Output_t &u, int state_count, bool in_landing_) {
  if (state_count == 1) // manual control
  {
    velocity_controller_x.init();
    velocity_controller_y.init();
    velocity_controller_z.init();
    u.acc[0] = 0;
    u.acc[1] = 0;
    u.acc[2] = -1;
  } else if (state_count == 2) // hover with rc
  {
    if (pos_vel_control_counter % 2 == 0) {
      desire_v_x = position_controller_gain_horizontal *
                   (des.p[0] - (odom.p[0] - initial_odom_position[0]));
      desire_v_y = position_controller_gain_horizontal *
                   (des.p[1] - (odom.p[1] - initial_odom_position[1]));
      desire_v_z = position_controller_gain_vertical *
                   (des.p[2] - (odom.p[2] - initial_odom_position[2]));
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
      pos_vel_control_counter = 0;
    } else {
      des_acc[0] = velocity_controller_x.update(desire_v_x - odom.v[0]);
      des_acc[1] = velocity_controller_y.update(desire_v_y - odom.v[1]);
      des_acc[2] = velocity_controller_z.update(desire_v_z - odom.v[2]);
    }
    pos_vel_control_counter++;
  } else // cmd control
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
  calculateThrustandAcc(u, des_acc);
  double roll, pitch;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  // cout << "yaw_odom: " << yaw_odom << endl;
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);
  roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
  pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
  Eigen::Quaterniond q_des =
      Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = q_des; // MAP(ENU)->IMU(FLU) * IMU(FLU)->MOCAP(FLU) *
               // MOCAP(FLU)->UAV_DES(FLU) = MAP(ENU)->UAV_DES(FLU)
  u.yaw = des.yaw;
  if (flight_state == FLYING && state_count == 2) {
    timed_thrust_.push(
        std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  }
  while (timed_thrust_.size() > 100) {
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
    const Eigen::Vector3d &des_acc) {
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a,
                                        const Parameter_t &param) {
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1) {
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
    if (thr > 0.01 && flight_state == FLYING) {
      /***********************************/
      /* Model: est_a(2) = thr1acc_ * thr */
      /***********************************/
      double gamma = 1 / (rho2_ + thr * P_ * thr);
      double K = gamma * P_ * thr;
      thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
      P_ = (1 - K * thr) * P_ / rho2_;
    }
    cout << thr2acc_ << endl;
    return true;
  }
  return false;
}

void LinearControl::resetThrustMapping(void) {
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
