#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <random>
class Second_order_system
{
public:
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
    Second_order_system(double x1_, double x2_, double u_)
        : x1(x1_), x2(x2_), u(u_), gen(rd()), dis(0.0, 0.1)
    {
        A << 0, 1,
            0, 0;
        B << 0,
            1;
        C << 1, 0;
        D = 0;
        // Random number generation setup
    }

    double generate_random()
    {
        return dis(gen); // Generate and return a random number
    }

    void update(double dt)
    {
        // State-space equations: x_dot = A * x + B * u
        Eigen::Vector2d x(x1, x2);
        // Compute the disturbance d as a sinusoidal function
        double amplitude = 1; // Amplitude of the disturbance
        double frequency = 1; // Frequency of the disturbance
        double phase = 0.0;   // Phase of the disturbance
        double d = amplitude * sin(2 * M_PI * frequency * ros::Time::now().toSec() + phase);
        auto f = [&](const Eigen::Vector2d &x, double u, double d)
        {
            return A * x + B * (u + d);
        };

        // RK4 integration
        Eigen::Vector2d k1 = f(x, u, d) * dt;
        Eigen::Vector2d k2 = f(x + 0.5 * k1, u, d) * dt;
        Eigen::Vector2d k3 = f(x + 0.5 * k2, u, d) * dt;
        Eigen::Vector2d k4 = f(x + k3, u, d) * dt;

        x = x + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;

        // Update the states
        x1 = x(0);
        x2 = x(1);
    }

    void input_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        u = msg->data;
    }

    void print_state()
    {
        std::cout << "x1: " << x1 << ", x2: " << x2 << std::endl;
    }
    double x1;
    double x2;
    double u;
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::RowVector2d C;
    double D;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "second_order_system");
    ros::NodeHandle nh;

    double dt = 0.001;
    Second_order_system second_order_system(1, 0, 0);

    ros::Subscriber sub = nh.subscribe("/input_x_axis", 1, &Second_order_system::input_callback, &second_order_system);
    ros::Publisher x_pub = nh.advertise<std_msgs::Float64MultiArray>("/x_state", 1);
    int count = 0;
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok())
    {
        second_order_system.update(dt);
        second_order_system.print_state();
        if (count != 0 && count % 50 == 0)
        {
            std_msgs::Float64MultiArray x_msg;
            x_msg.data.resize(2); // Ensure there is space for two elements
            x_msg.data[0] = second_order_system.x1;
            x_msg.data[1] = second_order_system.x2;
            x_pub.publish(x_msg);
            count = 0;
        }
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
