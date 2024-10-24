#include "mpai_qp.h"

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
    // 构建矩阵 M1 到 M4
    MatrixXd M1(1, 4);
    M1 << 0, 0, e1, 1;

    MatrixXd M2(1, 4);
    M2 << 1, 0, -1, 0;

    MatrixXd M3(1, 4);
    M3 << 0, 0, 0, e1;

    MatrixXd M4(1, 4);
    M4 << 0, 1, 0, -1;

    MatrixXd M5(4, 4);
    M5 = 0.5 * precice_w1 * M1.transpose() * M1 +
         0.5 * precice_z1 * M2.transpose() * M2 +
         0.5 * precice_w2 * M3.transpose() * M3 +
         0.5 * precice_z2 * M4.transpose() * M4;

    MatrixXd M6 = kroneckerProduct(MatrixXd::Identity(Np + 1, Np + 1), M5);

    int s = 2;
    int num_block_rows = 2 * Np + 2;
    int num_block_cols = 2 * Np + 2;

    MatrixXd M7 = MatrixXd::Zero(s * num_block_rows, s * num_block_cols);
    MatrixXd I = MatrixXd::Identity(s, s);

    // 奇数块
    for (int i_block = 1; i_block <= 2 * Np + 1; i_block += 2)
    {
        int j_block = (i_block + 1) / 2;
        int row_start = (i_block - 1) * s;
        int col_start = (j_block - 1) * s;
        M7.block(row_start, col_start, s, s) = I;
    }

    // 偶数块
    for (int i_block = 2; i_block <= 2 * Np + 2; i_block += 2)
    {
        int j_block = Np + (i_block + 2) / 2;
        int row_start = (i_block - 1) * s;
        int col_start = (j_block - 1) * s;
        M7.block(row_start, col_start, s, s) = I;
    }

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
    // cout << Phi << endl;
    MatrixXd T2 = MatrixXd::Identity((Np + 1) * 2, (Np + 1) * 2);
    MatrixXd T1(Phi.rows(), Phi.cols() + T2.cols());
    T1 << Phi, -T2;
    MatrixXd M8 = MatrixXd::Zero(2 * (Np + 1) * 2, Np + 2 * (Np + 1));
    M8.block(0, 0, 2 * (Np + 1), Np) = Phi;
    M8.block(2 * (Np + 1), Np, 2 * (Np + 1), 2 * (Np + 1)) = MatrixXd::Identity(2 * (Np + 1), 2 * (Np + 1));

    MatrixXd M9 = MatrixXd::Zero(Np + 2 * (Np + 1), Np + 2 * (Np + 1));
    M9.block(0, 0, Np, Np) = MatrixXd::Identity(Np, Np);
    bigMatrix1 = 2 * M8.transpose() * M7.transpose() * M6 * M7 * M8 + M9 * precice_z_u;
    //  Compute bigMatrix2 using Eigen
    MatrixXd R = MatrixXd::Zero(2, 2);
    R(0, 0) = precice_z1;
    R(1, 1) = precice_z2;
    MatrixXd bigR = MatrixXd::Zero(2 * (Np + 1), 2 * (Np + 1));
    for (int i = 0; i < Np + 1; ++i)
    {
        bigR.block(2 * i, 2 * i, 2, 2) = R;
    }
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
    // cout << q << endl;
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
    solver.settings()->setAbsoluteTolerance(1e-4); // 设置绝对误差阈值
    solver.settings()->setRelativeTolerance(1e-4); // 设置相对误差阈值
    solver.settings()->setMaxIteration(100);       // 设置最大迭代次数为100
    solver.settings()->setPolish(false);            // 启用解的后处理
    solver.settings()->setScaling(10);             // 设置缩放因子
    P = bigMatrix1.sparseView();
    // cout << bigMatrix1 << endl;
    // cout << P << endl;
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
    // cout << l << endl;
    // cout << u << endl;
    //  构建 A 矩阵
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
