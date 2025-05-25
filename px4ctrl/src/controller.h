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

class LinearControl
{
public:
	friend class PX4CtrlFSM;
	explicit LinearControl(Parameter_t &param);
	quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, Controller_Output_t &u);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	// Member variables
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;

	void handleManualControl(Controller_Output_t &u);

	void handleCommandControl(Controller_Output_t &u, const Desired_State_t &des);

	void updateDebugMsg(const Desired_State_t &des, const Controller_Output_t &u);
};
#endif // CONTROLLER_H_