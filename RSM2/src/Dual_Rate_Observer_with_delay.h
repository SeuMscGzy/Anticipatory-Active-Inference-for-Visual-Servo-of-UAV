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

using namespace std;
using namespace Eigen;

class DR0D
{
public:
    // Constructor to initialize variables
    DR0D(ros::NodeHandle &nh);

    // Method to run the RMS calculation loop
    void run(double measure_with_delay);

private:
    Matrix2d A;          // State matrix
    Vector2d L;                // Feedback matrix
    double T_s;                // Sample time
    double T_delay;            // Delay time
    double T_c;                // Control time
    int Np;                    // Number of steps
    int N_delay;               // Number of delay steps
    int N_minus;               // Np - N_delay
    vector<double> yp;         // Vector for intermediate calculations
    vector<Vector2d> z_past;   // Past states
    vector<Vector2d> z_future; // Future states
    friend class RSM_using_DROD_;

    // Method to reset vectors for the next iteration
    void resetVectors();
    void resetVectors_past();
};

#endif // RMS_H