/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>
#include "input.h"

struct Desired_State_t
{
	Vector3d p;
	Vector3d v;
	Vector3d a;
	Vector3d j;
	Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t() {};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p), v(Vector3d::Zero()), a(Vector3d::Zero()),
		  j(Vector3d::Zero()), q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0) {};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Quaterniond q;

	// Body rates in body frame
	Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	Vector3d vel; // [rad/s]

	Vector3d acc;

	double yaw;

	bool use_attitude_or_acc;
	// Vector3d des_v_real;
};

class PIDController
{
public:
	friend class LinearControl;
	double Kp, Ki, Kd;
	double integral_error;
	const double dt = 0.02; // 速度控制循环的时间间隔，50Hz

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
	void init() { integral_error = 0; }
};

class LinearControl
{
public:
	LinearControl(Parameter_t &);
	enum FlightState
	{
		GROUND,
		SLOW_START,
		FLYING,
		LANDING
	};

	FlightState flight_state = GROUND;
	Vector3d des_acc = Vector3d(0, 0, 0);
	bool first_time_in_function = true;
	Vector3d initial_odom_position = Vector3d(0, 0, 0);
	int satisfy_times_takeoff = 0;
	int satisfy_times_on_ground = 0;
	double altitude_diff = 0;
	int pos_vel_control_counter = 0;	 // 位置速度控制器的计数器
	PIDController velocity_controller_x; // x轴速度环PID参数
	PIDController velocity_controller_y; // y轴速度环PID参数
	PIDController velocity_controller_z; // z轴速度环PID参数
	double desire_v_x = 0;
	double desire_v_y = 0;
	double desire_v_z = 0;
	const double position_controller_gain_horizontal = 0.9; // 水平位置比环例控制增益
	const double position_controller_gain_vertical = 1;		// 水平位置比环例控制增益
	const double MIN_THRUST = 0.01;
	const double MAX_THRUST = 0.95;				  // 根据实际最大推力调整
	const double GROUND_ALTITUDE_THRESHOLD = 0.2; // 离地高度阈值，单位：米
	double slow_start_step = 0;
	double land_step = 0;
	quadrotor_msgs::Px4ctrlDebug
	calculateControl(const Desired_State_t &des, const Odom_Data_t &odom,
					 const Imu_Data_t &imu, Controller_Output_t &u,
					 int state_count, bool in_landing_);
	void updateFlightState(const Desired_State_t &des, const Odom_Data_t &odom,
						   int state_count, bool in_landing_);
	void calculateThrustandAcc(Controller_Output_t &u,
							   const Vector3d &des_acc);
	bool estimateThrustModel(const Vector3d &est_v,
							 const Parameter_t &param);
	void resetThrustMapping(void);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	double last_thrust = 0.01;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

	// Thrust-accel mapping params
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;
	double computeDesiredCollectiveThrustSignal(const Vector3d &des_acc);
	double fromQuaternion2yaw(Quaterniond q);
};

#endif