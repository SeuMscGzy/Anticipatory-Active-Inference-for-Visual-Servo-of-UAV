#include "mpai_qp_ty.h"

Optimizer::Optimizer(double dt, int Np, double precice_z1, double precice_z2,
                     double precice_w1, double precice_w2, double precice_z_u, double e1,
                     double umin, double umax)
    : dt(dt), Np(Np), precice_z1(precice_z1), precice_z2(precice_z2),
      precice_w1(precice_w1), precice_w2(precice_w2), precice_z_u(precice_z_u), e1(e1),
      umin(umin), umax(umax)
{
    bigMatrix1.resize(Np + 2 * (Np + 1), Np + 2 * (Np + 1));
    bigMatrix2.resize(1, Np + 2 * (Np + 1));
    // 离散系统矩阵
    MatrixXd Ad(2, 2);
    Ad << 1, dt,
        0, 1;
    MatrixXd Bd(2, 1);
    Bd << 0.5 * dt * dt,
        dt;

    // 定义 N x N+1 的矩阵 S_temp
    MatrixXd S_temp = MatrixXd::Zero(Np, Np + 1);
    for (int i = 0; i < Np; ++i)
    {
        S_temp(i, i + 1) = 1;
    }
    // 定义 2x2 的单位矩阵 I2
    Matrix2d I2 = Matrix2d::Identity();
    // 计算 Kronecker 积 S = S_temp ⊗ I2
    MatrixXd S = kroneckerProduct(S_temp, I2);

    // 定义 N x N+1 的矩阵 M_temp
    MatrixXd M_temp = MatrixXd::Zero(Np, Np + 1);
    for (int i = 0; i < Np; ++i)
    {
        M_temp(i, i) = 1;
    }
    // 计算 Kronecker 积 S = M_temp ⊗ I2
    Eigen::MatrixXd M = Eigen::kroneckerProduct(M_temp, I2);
    double prop = exp(-e1 * dt);
    M = prop * M;

    // 定义F矩阵
    MatrixXd F = MatrixXd::Identity(2, 2);
    MatrixXd F_temp = Ad;
    for (int i = 1; i <= Np; ++i)
    {
        F.conservativeResize(F.rows() + 2, F.cols());
        F.block(2 * i, 0, 2, 2) = F_temp;
        F_temp *= Ad;
    }
    // cout << F << endl;
    //   初始化 Phi 矩阵
    MatrixXd Phi((Np + 1) * 2, Np);
    Phi.setZero();
    // 构建 Phi 矩阵
    for (int i = 2; i <= Np + 1; ++i)
    {
        for (int j = 1; j < i; ++j)
        {
            int power = i - j - 1;

            // 计算 A 的幂
            MatrixXd A_power = MatrixXd::Identity(2, 2); // 初始化为单位矩阵
            for (int p = 1; p <= power; ++p)
            {
                A_power *= Ad;
            }

            // 计算块
            MatrixXd block = A_power * Bd;

            // 确定在 Phi 中的位置
            int row_start = (i - 1) * 2;
            int col_start = (j - 1);

            // 将块放入 Phi
            Phi.block(row_start, col_start, 2, 1) = block;
        }
    }

    //  Compute T1 and T2
    MatrixXd T1_temp = MatrixXd::Identity((Np + 1) * 2, (Np + 1) * 2);
    MatrixXd T1(Phi.rows(), Phi.cols() + T1_temp.cols());
    T1 << Phi, -T1_temp;
    MatrixXd T2 = MatrixXd::Identity(Np + (Np + 1) * 2, Np + (Np + 1) * 2);
    MatrixXd T2_temp1 = MatrixXd::Zero(2 * (Np + 1), Np);
    MatrixXd T2_temp2 = MatrixXd::Identity(2 * (Np + 1), 2 * (Np + 1));
    T2 << T2_temp1, T2_temp2;

    //  Compute bigR using Eigen
    MatrixXd R = MatrixXd::Zero(2, 2);
    R(0, 0) = precice_z1;
    R(1, 1) = precice_z2;
    MatrixXd bigR = MatrixXd::Zero(2 * (Np + 1), 2 * (Np + 1));
    for (int i = 0; i < Np + 1; ++i)
    {
        bigR.block(2 * i, 2 * i, 2, 2) = R;
    }

    //  Compute bigQ using Eigen
    MatrixXd Q = MatrixXd::Zero(2, 2);
    Q(0, 0) = precice_w1;
    Q(1, 1) = precice_w2;
    MatrixXd bigQ = MatrixXd::Zero(2 * Np, 2 * Np);
    for (int i = 0; i < Np; ++i)
    {
        bigQ.block(2 * i, 2 * i, 2, 2) = Q;
    }
    MatrixXd S_M = S - M;
    bigMatrix1 = T1.transpose() * bigR * T1 + T2.transpose() * S_M.transpose() * bigQ * S_M * T2;
    MatrixXd M13 = MatrixXd::Zero(Np + 2 * (Np + 1), Np + 2 * (Np + 1));
    M13.block(0, 0, Np, Np) = MatrixXd::Identity(Np, Np);
    bigMatrix1 = bigMatrix1 + M13 * precice_z_u;

    MatrixXd bigMatrix2_ = F.transpose() * bigR * T1;
    bigMatrix2 = bigMatrix2_;
    // Initialize the solver matrices
    initsolver();
}

vector<double> Optimizer::optimize(double x1_init_, double x2_init_, double mu_init_, double mu_p_init_)
{
    Eigen::RowVectorXd rowVec(2);
    rowVec << x1_init_, x2_init_;
    q = (rowVec * bigMatrix2).transpose();
    cout << q << endl;
    l[Np] = mu_init_;
    u[Np] = mu_init_;
    l[Np + 1] = mu_p_init_;
    u[Np + 1] = mu_p_init_;

    // 更新求解器的梯度和边界
    solver.updateGradient(q);
    solver.updateLowerBound(l);
    solver.updateUpperBound(u);

    // 求解更新后的问题
    auto sol_result = solver.solveProblem();
    if (sol_result == OsqpEigen::ErrorExitFlag::NoError)
    {
        Eigen::VectorXd solution = solver.getSolution();

        // Extracting u which has Np elements
        double u_now = solution(0);

        // Extracting mu which has Np + 1 elements
        double mu_next = solution(Np + 2);

        // Extracting mu_p which also has Np + 1 elements
        double mu_p_next = solution(Np + 3);

        // 计算二次形式部分
        double quadraticPart = 0.5 * (solution.transpose() * bigMatrix1 * solution).value();

        // 计算线性形式部分
        double linearPart = (q.transpose() * solution).value();

        // 计算总代价
        double cost = quadraticPart + linearPart;

        vector<double> result(4);
        result[0] = u_now;
        result[1] = mu_next;
        result[2] = mu_p_next;
        result[3] = cost;
        return result;
    }
    else
    {
        std::cerr << "Failed to solve the problem" << std::endl;
        vector<double> result(3);
        result[0] = 0;
        result[1] = 0;
        result[2] = 0;
        return result;
    }
}

void Optimizer::initsolver()
{
    // 初始化求解器
    solver.data()->setNumberOfVariables(Np + 2 * (Np + 1));
    solver.data()->setNumberOfConstraints(Np + 2 * (Np + 1));
    P = bigMatrix1.sparseView();
    cout << bigMatrix1 << endl;
    cout << P << endl;
    Eigen::VectorXd q1(Np + 2 * (Np + 1));
    q = q1;
    Eigen::SparseMatrix<double> A1(Np + 2 * (Np + 1), Np + 2 * (Np + 1));
    A = A1;
    Eigen::VectorXd l1(Np + 2 * (Np + 1)), u1(Np + 2 * (Np + 1));
    l = l1;
    u = u1;
    // 设置上下界
    l.head(Np).setConstant(umin);
    u.head(Np).setConstant(umax);
    l.segment(Np, Np + 2 * (Np + 1) - Np).setConstant(-std::numeric_limits<double>::infinity());
    u.segment(Np, Np + 2 * (Np + 1) - Np).setConstant(std::numeric_limits<double>::infinity());

    // 设置额外变量的精确值
    l[Np] = 0;
    u[Np] = 0;
    l[Np + 1] = 0;
    u[Np + 1] = 0;
    cout << l << endl;
    cout << u << endl;
    // 构建 A 矩阵
    A.setIdentity(); // 设为单位矩阵，因为我们希望每个变量独立
    // solver.data()->setNumberOfVariables(Np + 2 * (Np + 1));
    // solver.data()->setNumberOfConstraints(Np + 2 * (Np + 1));
    //  配置求解器
    if (!solver.data()->setHessianMatrix(P) ||
        !solver.data()->setGradient(q) ||
        !solver.data()->setLinearConstraintsMatrix(A) ||
        !solver.data()->setLowerBound(l) ||
        !solver.data()->setUpperBound(u))
    {
        std::cerr << "Problem setting solver data!" << std::endl;
        return; // 或其他错误处理
    }
    solver.initSolver();
}
