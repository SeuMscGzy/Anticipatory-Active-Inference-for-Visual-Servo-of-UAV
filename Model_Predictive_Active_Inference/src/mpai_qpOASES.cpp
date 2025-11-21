#include "mpai_qpOASES.h"

#include <iostream>

USING_NAMESPACE_QPOASES

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::kroneckerProduct;

Optimizer::Optimizer(double dt, int Np,
                     double precice_z1, double precice_z2,
                     double precice_w1, double precice_w2,
                     double precice_z_u,
                     double e1,
                     double umin, double umax)
    : dt_(dt),
      Np_(Np),
      precice_z1_(precice_z1),
      precice_z2_(precice_z2),
      precice_w1_(precice_w1),
      precice_w2_(precice_w2),
      precice_z_u_(precice_z_u),
      e1_(e1),
      umin_(umin),
      umax_(umax),
      nV_(3 * Np + 6 * (Np + 1)),
      qpSolver_(nV_),
      state_row_(Eigen::Matrix<double, 1, 6>::Zero()),
      bigMatrix1_(),
      bigMatrix2_(),
      bigMatrix3_(),
      xLowerTemplate_(VectorXd::Zero(nV_)),
      xUpperTemplate_(VectorXd::Zero(nV_)),
      xLowerScratch_(VectorXd::Zero(nV_)),
      xUpperScratch_(VectorXd::Zero(nV_)),
      solutionScratch_(VectorXd::Zero(nV_)),
      q_(VectorXd::Zero(nV_)),
      hessianRowMajor_(nV_, nV_)
{
    // ================= 1. 离散系统矩阵 =================
    MatrixXd Ad(6, 6);
    Ad << 1, 0, 0, dt_, 0, 0,
          0, 1, 0, 0,  dt_, 0,
          0, 0, 1, 0,  0,   dt_,
          0, 0, 0, 1,  0,   0,
          0, 0, 0, 0,  1,   0,
          0, 0, 0, 0,  0,   1;

    MatrixXd Bd(6, 3);
    Bd << -0.5 * dt_ * dt_, 0,                  0,
           0,                -0.5 * dt_ * dt_,  0,
           0,                 0,               -0.5 * dt_ * dt_,
          -dt_,               0,               0,
           0,                -dt_,             0,
           0,                 0,              -dt_;
    Bd = -Bd; // 你原来的写法，这里保持不变

    // 闭环矩阵 Closed_A = Ad - K
    MatrixXd K(6, 6);
    K << 0.8, 0,   0,   0,    0,    0,
         0,   0.8, 0,   0,    0,    0,
         0,   0,   0.8, 0,    0,    0,
         0,   0,   0,   0.15, 0,    0,
         0,   0,   0,   0,    0.15, 0,
         0,   0,   0,   0,    0,    0.15;
    MatrixXd Closed_A = Ad - K;

    VectorXd zr(6);
    zr.setZero();
    VectorXd kr = K * zr;              // 当前为 0
    VectorXd kr_temp = VectorXd::Ones(Np_);
    MatrixXd kr_full = kroneckerProduct(kr_temp, kr); // (6*Np x 1)，目前全 0

    // ================= 2. S, M, T1, T2, T3 =================
    MatrixXd S_temp = MatrixXd::Zero(Np_, Np_ + 1);
    for (int i = 0; i < Np_; ++i)
        S_temp(i, i + 1) = 1.0;

    MatrixXd I6 = MatrixXd::Identity(6, 6);
    MatrixXd S = kroneckerProduct(S_temp, I6); // (6*Np x 6*(Np+1))

    MatrixXd M_temp = MatrixXd::Zero(Np_, Np_ + 1);
    for (int i = 0; i < Np_; ++i)
        M_temp(i, i) = 1.0;

    MatrixXd M = kroneckerProduct(M_temp, Closed_A); // (6*Np x 6*(Np+1))

    // F: 堆叠 Ad^0, Ad^1, ..., Ad^Np
    MatrixXd F(6 * (Np_ + 1), 6);
    F.block(0, 0, 6, 6) = MatrixXd::Identity(6, 6);
    MatrixXd A_power = Ad;
    for (int i = 1; i <= Np_; ++i)
    {
        F.block(6 * i, 0, 6, 6) = A_power;
        A_power = A_power * Ad;
    }

    // Phi: (Np+1)*6 x 3*Np
    MatrixXd Phi((Np_ + 1) * 6, 3 * Np_);
    Phi.setZero();
    for (int i = 2; i <= Np_ + 1; ++i)
    {
        for (int j = 1; j < i; ++j)
        {
            int power = i - j - 1;
            MatrixXd A_pow = MatrixXd::Identity(6, 6);
            for (int p = 0; p < power; ++p)
                A_pow *= Ad;

            MatrixXd block = A_pow * Bd;
            int row_start = (i - 1) * 6;
            int col_start = 3 * (j - 1);
            Phi.block(row_start, col_start, 6, 3) = block;
        }
    }

    // T11: blockdiag(I6,...,I6) 大小 (6*(Np+1) x 6*(Np+1))
    MatrixXd T11 = MatrixXd::Zero(6 * (Np_ + 1), 6 * (Np_ + 1));
    for (int i = 0; i <= Np_; ++i)
        T11.block(6 * i, 6 * i, 6, 6) = MatrixXd::Identity(6, 6);

    MatrixXd T1(Phi.rows(), Phi.cols() + T11.cols());
    T1 << Phi, -T11;   // (6*(Np+1) x nV)

    // T2
    MatrixXd T2_temp1 = MatrixXd::Zero(6 * (Np_ + 1), 3 * Np_);
    MatrixXd T2_temp2 = MatrixXd::Identity(6 * (Np_ + 1), 6 * (Np_ + 1));
    MatrixXd T2(T2_temp1.rows(), T2_temp1.cols() + T2_temp2.cols());
    T2 << T2_temp1, T2_temp2;

    // T3
    MatrixXd T3_temp1 = MatrixXd::Identity(3 * Np_, 3 * Np_);
    MatrixXd T3_temp2 = MatrixXd::Zero(3 * Np_, 6 * (Np_ + 1));
    MatrixXd T3(T3_temp1.rows(), T3_temp1.cols() + T3_temp2.cols());
    T3 << T3_temp1, T3_temp2;

    // BigB
    MatrixXd BigB = kroneckerProduct(MatrixXd::Identity(Np_, Np_), Bd); // (6*Np x 3*Np)

    // ================= 3. 代价权重 bigR, bigQ =================
    MatrixXd R = MatrixXd::Zero(6, 6);
    R(0, 0) = precice_z1_;
    R(1, 1) = precice_z1_;
    R(2, 2) = precice_z1_;
    R(3, 3) = precice_z2_;
    R(4, 4) = precice_z2_;
    R(5, 5) = precice_z2_;
    MatrixXd C = MatrixXd::Identity(6, 6);
    R = C.transpose() * R * C;

    MatrixXd bigR = MatrixXd::Zero(6 * (Np_ + 1), 6 * (Np_ + 1));
    for (int i = 0; i <= Np_; ++i)
        bigR.block(6 * i, 6 * i, 6, 6) = R;

    MatrixXd Q = MatrixXd::Zero(6, 6);
    Q(0, 0) = precice_w1_;
    Q(1, 1) = precice_w1_;
    Q(2, 2) = precice_w1_;
    Q(3, 3) = precice_w2_;
    Q(4, 4) = precice_w2_;
    Q(5, 5) = precice_w2_;

    MatrixXd bigQ = MatrixXd::Zero(6 * Np_, 6 * Np_);
    for (int i = 0; i < Np_; ++i)
        bigQ.block(6 * i, 6 * i, 6, 6) = Q;

    MatrixXd S_M = S - M;

    // T4 = S_M*T2 - BigB*T3
    MatrixXd T4 = S_M * T2 - BigB * T3;    // (6*Np x nV_)

    // bigMatrix1 = T1' * bigR * T1 + T4' * bigQ * T4
    bigMatrix1_ = T1.transpose() * bigR * T1 + T4.transpose() * bigQ * T4;

    // M13: 只对前 3*Np 个控制加上 precice_z_u 权重（类似 R_u）
    MatrixXd M13 = MatrixXd::Zero(nV_, nV_);
    M13.block(0, 0, 3 * Np_, 3 * Np_) = MatrixXd::Identity(3 * Np_, 3 * Np_);
    bigMatrix1_ += M13 * precice_z_u_;

    // bigMatrix2 = F' * bigR * T1
    bigMatrix2_ = F.transpose() * bigR * T1;       // (6 x nV_)

    // bigMatrix3 = -kr_full' * bigQ * T4
    bigMatrix3_ = -kr_full.transpose() * bigQ * T4; // (1 x nV_)，当前为 0

    // 给 Hessian 赋值（行主序）
    hessianRowMajor_ = bigMatrix1_;

    // ================= 4. 初始化 bounds 模板 =================
    // 默认所有变量：近似无约束
    const double BIG = 1e10;
    xLowerTemplate_.setConstant(-BIG);
    xUpperTemplate_.setConstant( BIG);

    // 对前 3*Np 个控制量施加 umin/umax
    xLowerTemplate_.head(3 * Np_).setConstant(umin_);
    xUpperTemplate_.head(3 * Np_).setConstant(umax_);

    // 其它状态变量保持 [-BIG, BIG] 即可

    // 梯度初始化为 0
    q_.setZero();

    // 初始化 qpOASES
    initSolver();

    std::cout << "Optimizer initialized with qpOASES, nV = " << nV_ << std::endl;
}

std::vector<double> Optimizer::optimize(const Vector3d &x,
                                        const Vector3d &vx,
                                        const Vector3d &mu_init,
                                        const Vector3d &mu_p_init)
{
    // 填当前状态
    state_row_.head<3>() = x.transpose();
    state_row_.tail<3>() = vx.transpose();

    // q = (state_row * bigMatrix2 + bigMatrix3)'  → nV x 1
    RowVectorXd q_row = state_row_ * bigMatrix2_ + bigMatrix3_;
    q_ = q_row.transpose();

    // 上下界拷贝一份
    xLowerScratch_ = xLowerTemplate_;
    xUpperScratch_ = xUpperTemplate_;

    // 把当前 mu, mu_p 固定成等式约束（下界=上界）
    const int muIdx = 3 * Np_;
    xLowerScratch_.segment<3>(muIdx)      = mu_init;
    xUpperScratch_.segment<3>(muIdx)      = mu_init;
    xLowerScratch_.segment<3>(muIdx + 3)  = mu_p_init;
    xUpperScratch_.segment<3>(muIdx + 3)  = mu_p_init;

    int nWSR = 200;
    returnValue ret =
        qpSolver_.hotstart(q_.data(),
                           xLowerScratch_.data(),
                           xUpperScratch_.data(),
                           nWSR);

    if (ret != SUCCESSFUL_RETURN)
    {
        // hotstart 失败就重置并重新 init
        qpSolver_.reset();
        int nWSRInit = 1000;
        ret = qpSolver_.init(hessianRowMajor_.data(),
                             q_.data(),
                             xLowerScratch_.data(),
                             xUpperScratch_.data(),
                             nWSRInit);
    }

    if (ret != SUCCESSFUL_RETURN)
    {
        throw std::runtime_error("QP solve failed in Optimizer::optimize.");
    }

    // 取原始解
    qpSolver_.getPrimalSolution(solutionScratch_.data());

    double ux_now = solutionScratch_(0);
    double uy_now = solutionScratch_(1);
    double uz_now = solutionScratch_(2);

    double mu_x_next    = solutionScratch_(3 * Np_ + 6);
    double mu_y_next    = solutionScratch_(3 * Np_ + 7);
    double mu_z_next    = solutionScratch_(3 * Np_ + 8);
    double mu_p_x_next  = solutionScratch_(3 * Np_ + 9);
    double mu_p_y_next  = solutionScratch_(3 * Np_ + 10);
    double mu_p_z_next  = solutionScratch_(3 * Np_ + 11);

    double cost = 0.0;
    qpSolver_.getObjVal(&cost);

    std::vector<double> result = {
        ux_now, uy_now, uz_now,
        mu_x_next,   mu_y_next,   mu_z_next,
        mu_p_x_next, mu_p_y_next, mu_p_z_next,
        cost
    };
    return result;
}

void Optimizer::initSolver()
{
    Options options;
    options.boundTolerance    = 1e-4;
    options.epsRegularisation = 1e-4;
    options.printLevel        = PL_NONE; // 如需调试可改为 PL_MEDIUM
    qpSolver_.setOptions(options);

    int nWSR = 1000;
    returnValue ret =
        qpSolver_.init(hessianRowMajor_.data(),
                       q_.data(),
                       xLowerTemplate_.data(),
                       xUpperTemplate_.data(),
                       nWSR);

    if (ret != SUCCESSFUL_RETURN)
    {
        throw std::runtime_error("QP initialization failed in Optimizer::initSolver.");
    }
}
