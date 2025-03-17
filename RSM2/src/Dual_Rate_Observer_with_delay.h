#ifndef RMS_H
#define RMS_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <stack>
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
using namespace std;
using namespace Eigen;

class DR0D
{
public:
    // Constructor to initialize variables
    DR0D(ros::NodeHandle &nh, double T_sampling, double T_delay, double T_fast, double poles);

    // Method to run the RMS calculation loop
    void run(const Vector3d& measure_with_delay);
    void init(Vector3d measure_with_delay);
    
    // Method to reset vectors for the next iteration
    void resetVectors();
    void resetVectors_past();

private:
    Matrix6d A;          // State matrix
    Eigen::Matrix<double, 6, 3> L;                // Feedback matrix
    Eigen::Matrix<double, 3, 6> C1;                // Feedback matrix
    Eigen::Matrix<double, 3, 6> C2;                // Feedback matrix
    double T_s;                // Sample time
    double T_d;            // Delay time
    double T_c;                // Discrete calculation time
    double T_f;                // Fast time
    int N_p;                    // Number of steps
    int N_cal;                // Number of calculation steps
    int N_delay;               // Number of delay steps
    int N_minus;               // Number of steps minus delay steps
    double poles;               // poles of drod
    vector<Vector3d> yp;         // Vector for intermediate calculations
    vector<Vector3d> yp_bar;     // Vector for intermediate calculations
    vector<Vector6d> z_past;   // Past states
    vector<Vector6d> z_future; // Future states
    vector<Vector6d> z_future_dt; // Future states
    friend class RSM_using_DROD_;
};

#endif // RMS_H