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
    ros::init(argc, argv, "RMS");
    ros::NodeHandle nh("~");
    Vector2d C_bar;
    Matrix2d A_bar, A0;
    A_bar << 0.148975591137423, 0.00993170607582819,
        -12.1663399428895, 0.844195016445396;
    C_bar << 0.851024408862577,
        12.1663399428895;
    A0 << 1, 0.0200000000000000,
        0, 1;
    double T_s = 0.06;
    double T_c = 0.02;
    int Np = static_cast<int>(T_s / T_c);
    vector<VectorXd> z(Np);
    VectorXd z_temp(2);
    z_temp << 0, 0;
    for (int i = 0; i < Np; i++)
    {
        z[i] = z_temp;
    }
    double t = ros::Time::now().toSec();
    ros::Rate rate(50 / 3);
    while (ros::ok())
    {
        double a = ros::Time::now().toSec() - t;
        double b = a - 0.06;
        for (int i = 0; i < Np; i++)
        {
            if (i == 0)
            {
                z[0] = A_bar * z[2] + C_bar * b;
            }
            else
            {
                Vector2d coeff(1, 0);
                double predict_tag_x;
                predict_tag_x = coeff.transpose() * A0 * z[i - 1];
                //cout << predict_tag_x << endl;
                z[i] = A_bar * z[i - 1] + C_bar * predict_tag_x;
                //cout<<z[i](1)<<endl;
            }
        }
        // cout << a << " " << a + 0.02 << " " << a + 0.04 << endl;
        cout << z[0](0) - a << " " << z[1](0) - (a + 0.02) << " " << z[2](0) - (a + 0.04) << endl;
        rate.sleep();
    }
    return 0;
}
