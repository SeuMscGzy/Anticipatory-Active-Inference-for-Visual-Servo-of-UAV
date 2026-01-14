#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <casadi/casadi.hpp>
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>

using namespace casadi;

class Optimizer
{
public:
    Optimizer(double dt, int Np, double precice_z1, double precice_z2,
              double precice_w1, double precice_w2, double e1, double e2,
              double umin, double umax);
    double optimize(double x1_init_, double x2_init_, double mu_init_, double mu_p_init_);
    void initSolver();
    DM last_optimal_x1, last_optimal_x2, last_optimal_u, last_optimal_mu, last_optimal_mu_p;
    double dt;
    int Np;
    double precice_z1, precice_z2, precice_w1, precice_w2, e1, e2, umin, umax;
    double cost;
    MX x1, x2, u, mu, mu_p;
    Function solver;
    std::vector<double> lbg, ubg, lbx, ubx;
};

#endif // OPTIMIZER_H
