#include "mpai_qpOASES.h"
#include <iostream>
#include <qpOASES.hpp>
#include <Eigen/Sparse>
USING_NAMESPACE_QPOASES

Optimizer::Optimizer(double dt, int Np, double precice_z1, double precice_z2,
                     double precice_w1, double precice_w2, double precice_z_u,
                     double e1, double umin, double umax)
    : dt(dt), Np(Np), precice_z1(precice_z1), precice_z2(precice_z2),
      precice_w1(precice_w1), precice_w2(precice_w2), precice_z_u(precice_z_u),
      e1(e1), umin(umin), umax(umax),
      // 根据问题尺寸初始化 qpSolver 对象
      nV(3 * Np + 6 * (Np + 1)), qpSolver(nV) {
  // 离散系统矩阵
  MatrixXd Ad(6, 6);
  Ad << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  MatrixXd Bd(6, 3);
  Bd << -0.5 * dt * dt, 0, 0, 0, -0.5 * dt * dt, 0, 0, 0, -0.5 * dt * dt, -dt,
      0, 0, 0, -dt, 0, 0, 0, -dt;
  Bd = -Bd;
  Eigen::MatrixXd K(6, 6);
  K << 0.8, 0, 0, 0, 0, 0, 0, 0.8, 0, 0, 0, 0, 0, 0, 0.8, 0, 0, 0, 0, 0, 0, 0.15,
      0, 0, 0, 0, 0, 0, 0.15, 0, 0, 0, 0, 0, 0, 0.15;
  Eigen::MatrixXd Closed_A = Ad - K;
  Eigen::VectorXd zr(6);
  zr << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd kr = K * zr;
  Eigen::VectorXd kr_temp = Eigen::VectorXd::Ones(Np);
  Eigen::MatrixXd kr_full = kroneckerProduct(kr_temp, kr);

  MatrixXd S_temp = MatrixXd::Zero(Np, Np + 1);
  for (int i = 0; i < Np; ++i) {
    S_temp(i, i + 1) = 1;
  }

  // Define 2x2 identity matrix I2
  MatrixXd I2 = MatrixXd::Identity(6, 6);

  // Compute Kronecker product S = S_temp ⊗ I2
  MatrixXd S = kroneckerProduct(S_temp, I2);

  // Define Np x (Np + 1) matrix M_temp
  MatrixXd M_temp = MatrixXd::Zero(Np, Np + 1);
  for (int i = 0; i < Np; ++i) {
    M_temp(i, i) = 1;
  }

  // Compute Kronecker product M = M_temp ⊗ Closed_A
  MatrixXd M = kroneckerProduct(M_temp, Closed_A);

  // Compute F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd F_temp = Ad;
  for (int i = 1; i <= Np; ++i) {
    // Stack F_temp below F
    Eigen::MatrixXd F_new(F.rows() + F_temp.rows(), F.cols());
    F_new << F, F_temp;
    F = F_new;
    // Update F_temp to the next power of Ad
    F_temp = F_temp * Ad;
  }

  // Initialize Phi matrix
  Eigen::MatrixXd Phi((Np + 1) * 6, 3 * Np);
  Phi.setZero(); // Set all elements to zero
  // Construct Phi matrix
  for (int i = 2; i <= Np + 1; ++i) {
    for (int j = 1; j < i; ++j) {
      int power = i - j - 1;

      // Compute A to the power of 'power'
      Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(6, 6);
      for (int p = 0; p < power; ++p) {
        A_power *= Ad;
      }

      // Compute block = A_power * Bd
      Eigen::MatrixXd block = A_power * Bd;

      // Determine the position in Phi
      int row_start = (i - 1) * 6;
      int col_start = 3 * (j - 1);

      // Place block into Phi
      Phi.block(row_start, col_start, 6, 3) = block;
    }
  }

  // Compute T1 and T2
  Eigen::MatrixXd T11((6 * Np + 6), (6 * Np + 6));
  T11.setZero();
  // Fill T11 with identity matrices along the diagonal
  for (int i = 0; i <= Np; ++i) {
    T11.block(6 * i, 6 * i, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
  }
  MatrixXd T1(Phi.rows(), Phi.cols() + T11.cols());
  T1 << Phi, -T11;

  // Initialize and construct T2
  Eigen::MatrixXd T2_temp1 = Eigen::MatrixXd::Zero(6 * (Np + 1), 3 * Np);
  Eigen::MatrixXd T2_temp2 =
      Eigen::MatrixXd::Identity(6 * (Np + 1), 6 * (Np + 1));
  Eigen::MatrixXd T2(T2_temp1.rows(), T2_temp1.cols() + T2_temp2.cols());
  T2 << T2_temp1, T2_temp2;

  // Initialize and construct T3
  Eigen::MatrixXd T3_temp1 = Eigen::MatrixXd::Identity(3 * Np, 3 * Np);
  Eigen::MatrixXd T3_temp2 = Eigen::MatrixXd::Zero(3 * Np, 6 * (Np + 1));
  Eigen::MatrixXd T3(T3_temp1.rows(), T3_temp1.cols() + T3_temp2.cols());
  T3 << T3_temp1, T3_temp2;

  // Compute BigB
  MatrixXd BigB = kroneckerProduct(MatrixXd::Identity(Np, Np), Bd);

  Eigen::MatrixXd R(6, 6);
  R.setZero();
  R(0, 0) = precice_z1;
  R(1, 1) = precice_z1;
  R(2, 2) = precice_z1;
  R(3, 3) = precice_z2;
  R(4, 4) = precice_z2;
  R(5, 5) = precice_z2;
  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6, 6);
  R = C.transpose() * R * C;

  // Initialize bigR matrix
  Eigen::MatrixXd bigR(6 * (Np + 1), 6 * (Np + 1));
  bigR.setZero();
  for (int i = 0; i <= Np; ++i) {
    bigR.block(6 * i, 6 * i, 6, 6) = R;
  }

  // Initialize Q matrix
  Eigen::MatrixXd Q(6, 6);
  Q.setZero();
  Q(0, 0) = precice_w1;
  Q(1, 1) = precice_w1;
  Q(2, 2) = precice_w1;
  Q(3, 3) = precice_w2;
  Q(4, 4) = precice_w2;
  Q(5, 5) = precice_w2;

  // Initialize bigQ matrix
  Eigen::MatrixXd bigQ(6 * Np, 6 * Np);
  bigQ.setZero();
  for (int i = 0; i < Np; ++i) {
    bigQ.block(6 * i, 6 * i, 6, 6) = Q;
  }

  Eigen::MatrixXd S_M = S - M;

  // Compute T4 = S_M * T2 - BigB * T3
  MatrixXd T4 = S_M * T2 - BigB * T3;

  // Compute bigMatrix1 = T1' * bigR * T1 + T4' * bigQ * T4
  bigMatrix1 = T1.transpose() * bigR * T1 + T4.transpose() * bigQ * T4;

  // Define M13 = zeros(Np + 2 * (Np + 1), Np + 2 * (Np + 1))
  MatrixXd M13 = MatrixXd::Zero(3 * Np + 6 * (Np + 1), 3 * Np + 6 * (Np + 1));

  // Set M13(0:Np - 1, 0:Np - 1) = Identity matrix
  M13.block(0, 0, 3 * Np, 3 * Np) = MatrixXd::Identity(3 * Np, 3 * Np);

  // Update bigMatrix1
  bigMatrix1 += M13 * precice_z_u;
  /*// Compute eigenvalues of bigMatrix1
  SelfAdjointEigenSolver<MatrixXd> eigensolver(bigMatrix1);
  if (eigensolver.info() != Success)
  {
      std::cerr << "Eigenvalue computation did not converge." << std::endl;
  }
  VectorXd eigenvalues = eigensolver.eigenvalues();*/

  bigMatrix2 = F.transpose() * bigR * T1;
  // cout << bigMatrix2 << endl;

  // Compute bigMatrix3 = -kr' * bigQ * T4
  bigMatrix3 = -kr_full.transpose() * bigQ * T4;

  // 初始化梯度向量 q 大小为 nV
  q = VectorXd::Zero(nV);

  // 初始化 qpOASES 求解器（构造时指定维度）
  initsolver();
  cout << "Optimizer initialized with qpOASES" << endl;
}

vector<double> Optimizer::optimize(Vector3d x, Vector3d vx, Vector3d mu_init_,
                                   Vector3d mu_p_init_) {
  // 根据当前状态计算梯度项 q
  RowVectorXd rowVec(6);
  rowVec << x.transpose(), vx.transpose();
  q = rowVec * bigMatrix2 + bigMatrix3; // 得到 nV 维的梯度向量

  // 创建一个 nV 维的边界向量（初始全部无界）
  VectorXd xLower = VectorXd::Constant(nV, -INFTY);
  VectorXd xUpper = VectorXd::Constant(nV, INFTY);

  // 前 3*Np 个决策变量保持 umin 和 umax 不变
  xLower.segment(0, 3 * Np) = VectorXd::Constant(3 * Np, umin);
  xUpper.segment(0, 3 * Np) = VectorXd::Constant(3 * Np, umax);

  // 对第 3*Np 到 3*Np+5 的决策变量，根据 mu_init_ 和 mu_p_init_ 更新边界
  xLower[3 * Np + 0] = mu_init_[0];
  xUpper[3 * Np + 0] = mu_init_[0];
  xLower[3 * Np + 1] = mu_init_[1];
  xUpper[3 * Np + 1] = mu_init_[1];
  xLower[3 * Np + 2] = mu_init_[2];
  xUpper[3 * Np + 2] = mu_init_[2];
  xLower[3 * Np + 3] = mu_p_init_[0];
  xUpper[3 * Np + 3] = mu_p_init_[0];
  xLower[3 * Np + 4] = mu_p_init_[1];
  xUpper[3 * Np + 4] = mu_p_init_[1];
  xLower[3 * Np + 5] = mu_p_init_[2];
  xUpper[3 * Np + 5] = mu_p_init_[2];
  int nWSR = 1000;
  int ret = qpSolver.hotstart(q.transpose().data(), xLower.data(),
                              xUpper.data(), nWSR);

  Eigen::VectorXd solution = Eigen::VectorXd::Zero(nV);
  qpSolver.getPrimalSolution(solution.data());

  double ux_now = solution(0);
  double uy_now = solution(1);
  double uz_now = solution(2);
  double mu_x_next = solution(3 * Np + 6);
  double mu_y_next = solution(3 * Np + 7);
  double mu_z_next = solution(3 * Np + 8);
  double mu_p_x_next = solution(3 * Np + 9);
  double mu_p_y_next = solution(3 * Np + 10);
  double mu_p_z_next = solution(3 * Np + 11);

  double cost;
  qpSolver.getObjVal(&cost);

  vector<double> result = {ux_now,      uy_now,    uz_now,      mu_x_next,
                           mu_y_next,   mu_z_next, mu_p_x_next, mu_p_y_next,
                           mu_p_z_next, cost};
  return result;
}

void Optimizer::initsolver() {
  Options options;
  // 提升精度：把边界、互补等容忍度都调小
  options.boundTolerance = 1e-4;
  options.epsRegularisation = 1e-4;
  options.printLevel = PL_NONE;
  qpSolver.setOptions(options);
  // 确保Hessian对称，并以行优先存储
  Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> H_row_major = bigMatrix1;

  // 变量上下界（示例，根据实际情况调整）
  VectorXd xLower = VectorXd::Constant(nV, -INFTY);
  VectorXd xUpper = VectorXd::Constant(nV, INFTY);
  // 初始化QP问题
  int nWSR = 1000; // 增加最大迭代次数
  returnValue ret = qpSolver.init(H_row_major.data(), q.data(), xLower.data(),
                                  xUpper.data(), nWSR);
  if (ret != SUCCESSFUL_RETURN) {
    // 错误处理，可尝试更详细的调试信息
    throw std::runtime_error("QP initialization failed.");
  }
}
