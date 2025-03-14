#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <cmath>
using namespace std;
using namespace Eigen;
class Optimizer
{
public:
    Optimizer(double dt, int Np_, double precice_z1, double precice_z2,
              double precice_w1, double precice_w2, double precice_z_u, double e1,
              double umin, double umax);

    vector<double> optimize(Eigen::Vector3d x, Eigen::Vector3d vx, Eigen::Vector3d mu_init_, Eigen::Vector3d mu_p_init_);
    void shiftDecisionVariables(VectorXd &solution, VectorXd &Dualsolution);

    OsqpEigen::Solver solver;

private:
    void initsolver();

    // Member variables
    double dt;
    int Np;
    double precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1;
    double umin, umax;

    // Matrices and vectors
    MatrixXd bigMatrix1;
    MatrixXd bigMatrix2;
    MatrixXd bigMatrix3;

    SparseMatrix<double> P;
    VectorXd q;
    SparseMatrix<double> A;
    VectorXd l, u;
};
