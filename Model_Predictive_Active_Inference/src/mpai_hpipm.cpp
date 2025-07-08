#include "mpai_hpipm.h"

Optimizer::Optimizer(double dt_, int Np_,
                     double precice_z1_, double precice_z2_,
                     double precice_w1_, double precice_w2_,
                     double precice_z_u_,
                     double e1_, double umin_, double umax_)
    : dt(dt_), Np(Np_), precice_z1(precice_z1_), precice_z2(precice_z2_),
      precice_w1(precice_w1_), precice_w2(precice_w2_), precice_z_u(precice_z_u_),
      e1(e1_), umin(umin_), umax(umax_)
{
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
  for (int i = 0; i < Np; ++i)
  {
    S_temp(i, i + 1) = 1;
  }

  // Define 2x2 identity matrix I2
  MatrixXd I2 = MatrixXd::Identity(6, 6);

  // Compute Kronecker product S = S_temp ⊗ I2
  MatrixXd S = kroneckerProduct(S_temp, I2);

  // Define Np x (Np + 1) matrix M_temp
  MatrixXd M_temp = MatrixXd::Zero(Np, Np + 1);
  for (int i = 0; i < Np; ++i)
  {
    M_temp(i, i) = 1;
  }

  // Compute Kronecker product M = M_temp ⊗ Closed_A
  MatrixXd M = kroneckerProduct(M_temp, Closed_A);

  // Compute F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd F_temp = Ad;
  for (int i = 1; i <= Np; ++i)
  {
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
  for (int i = 2; i <= Np + 1; ++i)
  {
    for (int j = 1; j < i; ++j)
    {
      int power = i - j - 1;

      // Compute A to the power of 'power'
      Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(6, 6);
      for (int p = 0; p < power; ++p)
      {
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
  for (int i = 0; i <= Np; ++i)
  {
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
  for (int i = 0; i <= Np; ++i)
  {
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
  for (int i = 0; i < Np; ++i)
  {
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

  nV = 3 * Np + 6 * (Np + 1);

  // allocate HPIPM structs
  long int sz;
  sz = d_dense_qp_dim_memsize();
  dim_mem_ = malloc(sz);
  d_dense_qp_dim_create(&dim_, dim_mem_);
  d_dense_qp_dim_set("nx", nV, &dim_);
  d_dense_qp_dim_set("nb", nV, &dim_);
  d_dense_qp_dim_set("ng", 0, &dim_);
  d_dense_qp_dim_set("n_eq", 0, &dim_);

  sz = d_dense_qp_memsize(&dim_);
  qp_mem_ = malloc(sz);
  d_dense_qp_create(&dim_, &qp_, qp_mem_);

  sz = d_dense_qp_ipm_arg_memsize(&dim_);
  arg_mem_ = malloc(sz);
  d_dense_qp_ipm_arg_create(&dim_, &arg_, arg_mem_);

  sz = d_dense_qp_ipm_ws_memsize(&dim_, &arg_);
  ws_mem_ = malloc(sz);
  d_dense_qp_ipm_ws_create(&dim_, &arg_, &ws_, ws_mem_);

  sz = d_dense_qp_sol_memsize(&dim_);
  sol_mem_ = malloc(sz);
  d_dense_qp_sol_create(&dim_, &sol_, sol_mem_);

  double tol = 1e-6;
  d_dense_qp_ipm_arg_set_tol_stat(&tol, &arg_);
  d_dense_qp_ipm_arg_set_tol_eq(&tol, &arg_);
  d_dense_qp_ipm_arg_set_tol_ineq(&tol, &arg_);
  d_dense_qp_ipm_arg_set_tol_comp(&tol, &arg_);

  // upload Hessian
  Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> Hrm = bigMatrix1;
  d_dense_qp_set_H(Hrm.data(), &qp_);

  cout << "Optimizer initialized with qpOASES" << endl;
}

Optimizer::~Optimizer()
{
  free(dim_mem_);
  free(qp_mem_);
  free(arg_mem_);
  free(ws_mem_);
  free(sol_mem_);
}

vector<double> Optimizer::optimize(Vector3d x, Vector3d vx, Vector3d mu_init,
                                   Vector3d mu_p_init)
{
  // a) gradient
  RowVectorXd rv(6);
  rv << x.transpose(), vx.transpose();
  q = rv * bigMatrix2 + bigMatrix3;
  d_dense_qp_set_g(q.data(), &qp_);

  // b) bounds
  std::vector<double> lb(nV, -1e20), ub(nV, +1e20);
  for (int i = 0; i < 3 * Np; ++i)
  {
    lb[i] = umin;
    ub[i] = umax;
  }
  for (int i = 0; i < 6; ++i)
  {
    double v = (i < 3 ? mu_init[i] : mu_p_init[i - 3]);
    lb[3 * Np + i] = ub[3 * Np + i] = v;
  }
  d_dense_qp_set_lb(lb.data(), &qp_);
  d_dense_qp_set_ub(ub.data(), &qp_);

  d_dense_qp_ipm_solve(&qp_, &sol_, &arg_, &ws_);
  // --- 从 sol_ 里取出 primal x ---
  std::vector<double> solx(nV);
  d_dense_qp_sol_get_v(&sol_, solx.data());
  // --- 取目标值 ---
  double cost;
  d_dense_qp_sol_get_obj(&sol_, &cost);

  // e) return
  std::vector<double> out = {solx[0], solx[1], solx[2],
                             solx[3 * Np + 6], solx[3 * Np + 7], solx[3 * Np + 8],
                             solx[3 * Np + 9], solx[3 * Np + 10], solx[3 * Np + 11], cost};
  return out;
}
