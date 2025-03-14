#ifndef MPAI_QP_H
#define MPAI_QP_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <qpOASES.hpp>
#include <qpOASES/QProblemB.hpp>
#include <vector>
using namespace std;
using namespace Eigen;
using namespace qpOASES;
class Optimizer {
public:
  // 构造函数中传入 dt、预测步长 Np 以及各权重和输入上下界等参数
  Optimizer(double dt, int Np, double precice_z1, double precice_z2,
            double precice_w1, double precice_w2, double precice_z_u, double e1,
            double umin, double umax);

  // 求解 QP 并返回一个包含控制量、后续 mu 以及总代价的 vector
  vector<double> optimize(Vector3d x, Vector3d vx, Vector3d mu_init,
                          Vector3d mu_p_init);

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

  // QP 数据：梯度向量，尺寸为 nV
  VectorXd q;

  // QP 问题维度：决策变量数（无额外约束）
  int nV; // = 3*Np + 6*(Np+1)

  // qpOASES 求解器对象
  QProblemB qpSolver;
};
#endif // MPAI_QP_H