// ======= mpai_hpipm.h =======
#pragma once

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <vector>
extern "C"
{
#include "hpipm_d_dense_qp_dim.h"
#include "hpipm_d_dense_qp.h"
#include "hpipm_d_dense_qp_ipm.h"
}
using namespace std;
using namespace Eigen;
class Optimizer
{
public:
  Optimizer(double dt, int Np,
            double precice_z1, double precice_z2,
            double precice_w1, double precice_w2,
            double precice_z_u,
            double e1,
            double umin, double umax);
  ~Optimizer();

  vector<double> optimize(Vector3d x, Vector3d vx, Vector3d mu_init,
                          Vector3d mu_p_init);

private:
  // parameters
  double dt;
  int Np;
  double precice_z1, precice_z2;
  double precice_w1, precice_w2;
  double precice_z_u;
  double e1;
  double umin, umax;
  int nV;

  // precomputed QP matrices
  MatrixXd bigMatrix1;
  MatrixXd bigMatrix2;
  MatrixXd bigMatrix3;
  // QP 数据：梯度向量，尺寸为 nV
  VectorXd q;

  // HPIPM objects
  d_dense_qp_dim dim_;
  d_dense_qp qp_;
  d_dense_qp_ipm_arg arg_;
  d_dense_qp_ipm_ws ws_;
  d_dense_qp_sol sol_;
  void *dim_mem_, *qp_mem_, *arg_mem_, *ws_mem_, *sol_mem_;
};