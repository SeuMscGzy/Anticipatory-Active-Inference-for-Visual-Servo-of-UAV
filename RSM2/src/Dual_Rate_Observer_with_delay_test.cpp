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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RSM");
    ros::NodeHandle nh("~");
    MatrixXd A(2, 2);
    A << 0, 1, 0, 0;
    VectorXd L(2);
    L << -20, -100;
    double T_s = 0.06;
    double T_delay = 0.06;
    double T_c = 0.01;
    int Np = static_cast<int>(T_s / T_c);
    int N_delay = static_cast<int>(T_delay / T_c);
    int N_minus = Np - N_delay;
    vector<double> yp(Np);
    vector<VectorXd> z_past(Np);
    vector<VectorXd> z_future(Np);
    VectorXd z_temp(2);
    z_temp << 0, 0;
    for (int i = 0; i < Np; i++)
    {
        yp[i] = 0;
        z_past[i] = z_temp;
        z_future[i] = z_temp;
    }
    double t = ros::Time::now().toSec();
    ros::Rate rate(50 / 3);
    while (ros::ok())
    {
        double a = ros::Time::now().toSec() - t;
        // double c = cos(ros::Time::now().toSec());
        double b = a - 0.06;
        z_future[0] = z_past[Np - 1];
        for (int i = 0; i < Np; i++)
        {
            yp[i] = b;
            for (int j = N_minus; j < Np; j++)
            {
                yp[i] += T_c * z_past[j](1);
            }
            for (int k = 0; k < i; k++)
            {
                yp[i] += T_c * z_future[k](1);
            }
            VectorXd z_temp(2);
            if (i == 0)
            {
                z_temp = z_future[i] + T_c * (A * z_future[i] - L * (yp[i] - z_future[i](0)));
                z_future[i] = z_temp;
            }
            else
            {
                z_temp = z_future[i - 1] + T_c * (A * z_future[i - 1] - L * (yp[i] - z_future[i - 1](0)));
                z_future[i] = z_temp;
            }
        }
        cout << "position tracking error of TDDRO in sampling step:" << z_future[0](0) - a << endl;
        cout << "velocity tracking error of TDDRO in sampling step:" << z_future[0](1) - 1 << endl;
        cout << "position tracking error of TDDRO in first prediction step:" << z_future[20](0) - a - 0.02 << endl;
        cout << "velocity tracking error of TDDRO in first prediction step:" << z_future[20](1) - 1 << endl;
        cout << "position tracking error of TDDRO in second prediction step:" << z_future[40](0) - a - 0.04 << endl;
        cout << "velocity tracking error of TDDRO in second prediction step:" << z_future[40](1) - 1 << endl;
        z_past = z_future;
        for (int i = 0; i < Np; i++)
        {
            VectorXd z_temp(2);
            z_temp << 0, 0;
            yp[i] = 0;
            z_future[i] = z_temp;
        }
        rate.sleep();
    }
    return 0;
}