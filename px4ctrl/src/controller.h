/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>
#include <cmath>
#include "input.h"
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <algorithm>
struct Desired_State_t
{
	Vector3d p;
	Vector3d v;
	Vector3d a;
	Vector3d j;
	Quaterniond q;
	double yaw;
	double yaw_rate;
	Desired_State_t() = default;
	explicit Desired_State_t(Odom_Data_t &odom)
		: p(odom.p), v(Vector3d::Zero()), a(Vector3d::Zero()),
		  j(Vector3d::Zero()), q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0) {}
};

struct Controller_Output_t
{
	Quaterniond q;
	Vector3d bodyrates;
	double thrust;
	Vector3d vel;
	Vector3d acc;
	double yaw;
	bool use_attitude_or_acc;
};

class PIDController
{
public:
	double Kp, Ki, Kd;
	double integral_error;
	static constexpr double kDt = 0.02; // 50Hz control loop
	PIDController(double Kp_, double Ki_, double Kd_)
		: Kp(Kp_), Ki(Ki_), Kd(Kd_), integral_error(0) {}
	double update(double error)
	{
		integral_error += error * kDt;
		integral_error = clamp(integral_error, -1.0, 1.0);
		double u_pid = Kp * error + Ki * integral_error;
		return clamp(u_pid, -2.0, 2.0);
	}
	void init() { integral_error = 0; }
};

class LinearControl
{
public:
	friend class PX4CtrlFSM;
	enum class FlightState
	{
		GROUND,
		SLOW_START,
		FLYING
	};
	// Constants
	static constexpr double kMinThrust = 0.01;
	static constexpr double kMaxThrust = 0.95;
	static constexpr double kGroundAltThreshold = 0.2;
	static constexpr double kPositionGainHorizontal = 0.9;
	static constexpr double kPositionGainVertical = 1.0;
	static constexpr int kSlowStartSteps = 200;
	static constexpr double kSlowStartFactor = 0.005;
	static constexpr double kRho2 = 0.998;
	explicit LinearControl(Parameter_t &param);
	quadrotor_msgs::Px4ctrlDebug calculateControl(
		const Desired_State_t &des, const Odom_Data_t &odom,
		const Imu_Data_t &imu, Controller_Output_t &u,
		int state_count);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	// Member variables
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	FlightState flight_state = FlightState::GROUND;

	PIDController velocity_controller_x_{2, 0.4, 0};
	PIDController velocity_controller_y_{2, 0.4, 0};
	PIDController velocity_controller_z_{4, 2, 0};

	Vector3d des_acc = Vector3d::Zero();
	double des_acc_SE3 = 0.0;				  // Desired acceleration in SE(3) space
	Matrix3d R_w2i = Matrix3d::Identity();	  // Rotation from world to inertial frame
	Vector3d b3_in_world = Vector3d::UnitZ(); // Body z-axis in world frame
	Vector3d initial_odom_position = Vector3d::Zero();
	double last_thrust_ = kMinThrust;
	double altitude_diff_ = 0.0;

	int satisfy_times_takeoff_ = 0;
	int satisfy_times_on_ground_ = 0;
	int pos_vel_control_counter_ = 0;
	int slow_start_step_ = 0;

	double desire_v_x = 0;
	double desire_v_y = 0;
	double desire_v_z_ = 0;

	double thr2acc;
	double P;
	queue<pair<ros::Time, double>> timed_thrust_;
	bool first_time_in_function = true;
	// Private methods
	double computeDesiredCollectiveThrustSignal();
	double fromQuaternion2yaw(const Quaterniond &q);
	bool estimateThrustModel(const Vector3d &est_a, const Parameter_t &param);
	void resetThrustMapping();

	// Control mode handlers
	void updateFlightState(const Desired_State_t &des, const Odom_Data_t &odom);
	void handleManualControl();
	void handleHoverControl(const Desired_State_t &des, const Odom_Data_t &odom);
	void handleCommandControl(const Desired_State_t &des);
	void calculateThrust(Controller_Output_t &u);
	// Utility functions
	void calculateAttitude(double yaw, const Odom_Data_t &odom, Controller_Output_t &u);
	void updateDebugMsg(const Desired_State_t &des, const Controller_Output_t &u);
};
#endif // CONTROLLER_H_