#include "mpai_casadi.h"

//动态写在代价函数里！！！！！！！！！！！！！

using namespace casadi;
using namespace std;

Optimizer::Optimizer(double dt, int Np, double precice_z1, double precice_z2,
                     double precice_w1, double precice_w2, double e1, double e2,
                     double umin, double umax)
    : dt(dt), Np(Np), precice_z1(precice_z1), precice_z2(precice_z2),
      precice_w1(precice_w1), precice_w2(precice_w2), e1(e1), e2(e2),
      umin(umin), umax(umax), cost(0)
{
    last_optimal_x1 = DM::zeros(1, 1);
    last_optimal_x2 = DM::zeros(1, 1);
    last_optimal_u = DM::zeros(Np, 1);
    last_optimal_mu = DM::zeros(Np + 1, 1);
    last_optimal_mu_p = DM::zeros(Np + 1, 1);
    // Initialize the solver
    initSolver();
}

double Optimizer::optimize(double x1_init_, double x2_init_, double mu_init_, double mu_p_init_)
{
    std::map<std::string, DM> arg, res;
    if (last_optimal_x1.is_empty())
    {
        arg["x0"] = DM::zeros(x1.size1() + x2.size1() + u.size1() + mu.size1() + mu_p.size1());
    }
    else
    {
        DM x0 = DM::zeros(2 + Np + 2 * (Np + 1), 1);
        int idx = 0;

        // Update x1 element
        x0(idx) = last_optimal_x1;
        idx += 1;

        // Update x2 element
        x0(idx) = last_optimal_x2;
        idx += 1;

        // Update u elements (shifting forward by one position and copying the last element)
        x0(Slice(idx, idx + Np - 1)) = last_optimal_u(Slice(1, Np)); // Skip the first element of last_optimal_u
        x0(idx + Np - 1) = last_optimal_u(Np - 1);                   // Assume the last element can be repeated
        idx += Np;

        // Update mu elements (similar to shifting strategy)
        x0(Slice(idx, idx + Np)) = last_optimal_mu(Slice(1, Np + 1)); // Skip the first element of last_optimal_mu
        x0(idx + Np) = last_optimal_mu(Np);                           // Assume the last element can be repeated
        idx += Np + 1;

        // Update mu_p elements (similar to shifting strategy)
        x0(Slice(idx, idx + Np)) = last_optimal_mu_p(Slice(1, Np + 1)); // Skip the first element of last_optimal_mu_p
        x0(idx + Np) = last_optimal_mu_p(Np);                           // Assume the last element can be repeated
        idx += Np + 1;

        arg["x0"] = x0; // Assign the constructed initial guess to arg
    }
    // Update initial condition constraints
    // Set the bounds for x1 and x2
    lbx[0] = ubx[0] = x1_init_;
    lbx[1] = ubx[1] = x2_init_;

    // Set the bounds for mu
    // Index for mu starts after x1, x2, and u: 2 + Np
    lbx[2 + Np] = ubx[2 + Np] = mu_init_;

    // Set the bounds for mu_p
    // Index for mu_p starts after x1, x2, u, and mu: 2 + Np + (Np + 1)
    lbx[3 + 2 * Np] = ubx[3 + 2 * Np] = mu_p_init_;

    arg["lbx"] = DM(lbx);
    arg["ubx"] = DM(ubx);
    arg["lbg"] = DM(lbg);
    arg["ubg"] = DM(ubg);

    // auto start_time = std::chrono::high_resolution_clock::now(); // Start time for optimization
    res = solver(arg);
    DM x_opt = res["x"];
    DM cost_ = res["f"];   // 获取优化后的代价函数值
    cost = cost_(0).scalar(); // 代价函数值

    int idx = 0;

    // Extracting x1 which has 1 element
    last_optimal_x1 = x_opt(Slice(idx, idx + 1));
    idx += 1;

    // Extracting x2 which also has 1 element
    last_optimal_x2 = x_opt(Slice(idx, idx + 1));
    idx += 1;

    // Extracting u which has Np elements
    last_optimal_u = x_opt(Slice(idx, idx + Np));
    idx += Np;

    // Extracting mu which has Np + 1 elements
    last_optimal_mu = x_opt(Slice(idx, idx + Np + 1));
    idx += Np + 1;

    // Extracting mu_p which also has Np + 1 elements
    last_optimal_mu_p = x_opt(Slice(idx, idx + Np + 1));
    // cout<<"last_optimal_mu: "<<last_optimal_mu(0).scalar()<<endl;
    // cout<<"last_optimal_mu_p: "<<last_optimal_mu_p(0).scalar()<<endl;
    // cout<<last_optimal_x2(0).scalar()<<endl;
    // cout<<last_optimal_x2(1).scalar()<<endl;
    cout << last_optimal_x1 << endl;
    cout << last_optimal_x2 << endl;
    cout << last_optimal_mu << endl;
    cout << last_optimal_mu_p << endl;
    cout << last_optimal_u << endl;
    return last_optimal_u(0).scalar();
}

void Optimizer::initSolver()
{
    x1 = MX::sym("x1", 1);
    x2 = MX::sym("x2", 1);
    u = MX::sym("u", Np);
    mu = MX::sym("mu", Np + 1);
    mu_p = MX::sym("mu_p", Np + 1);

    MX objective = 0;
    MX x1_last = x1;
    MX x2_last = x2;
    for (int i = 0; i < Np; ++i)
    {
        x1_last = x1_last + dt * x2_last + 0.5 * dt * dt * u(i);
        x2_last = x2_last + dt * u(i);
        objective += 0.5 * precice_z1 * pow(x1_last - mu(i + 1), 2) +
                     0.5 * precice_z2 * pow(x2_last - mu_p(i + 1), 2) +
                     0.5 * precice_w1 * pow(mu_p(i + 1) + e1 * mu(i + 1), 2) +
                     0.5 * precice_w2 * e2 * e2 * pow(mu_p(i + 1), 2) +
                     0.5 * 0.004 * pow(u(i), 2);
    }
    std::vector<MX> g;
    // 假设已经计算了各个变量在决策向量中的起始位置
    int start_index_of_u = x1.size1() + x2.size1(); // 调整此处确保正确计算 u 的起始索引

    // 初始化 lbx 和 ubx
    lbx.resize(x1.size1() + x2.size1() + u.size1() + mu.size1() + mu_p.size1(), -casadi::inf);
    ubx.resize(x1.size1() + x2.size1() + u.size1() + mu.size1() + mu_p.size1(), casadi::inf);

    // 为 u 设置界限
    for (int i = start_index_of_u; i < start_index_of_u + u.size1(); ++i)
    {
        lbx[i] = umin; // 设置 u 的下界
        ubx[i] = umax; // 设置 u 的上界
    }
    MXDict nlp = {
        {"x", vertcat(x1, x2, u, mu, mu_p)}, // 所有决策变量
        {"f", objective},                    // 目标函数
        {"g", vertcat(g)},                   // 所有约束
    };
    Dict opts = {
        {"qpsol", "qpoases"},
        {"print_iteration", 0},
        {"print_header", 0},
        {"print_status", 0},
        {"qpsol_options.print_time", 0},
        {"qpsol_options.printLevel", "none"},
        {"max_iter", 100},
        {"tol_pr", 1e-3},
        {"tol_du", 1e-3},
        {"beta", 0.5}};

    solver = nlpsol("solver", "sqpmethod", nlp, opts);

    /*solver = nlpsol("solver", "ipopt",
                    {{"x", vertcat(x1, x2, u, mu, mu_p)}, {"f", objective}, {"g", vertcat(g)}},
                    {{"ipopt.tol", 1e-3},  // 设定Ipopt的选项
                     {"ipopt.max_iter", 100},  // 最大迭代次数
                     {"ipopt.linear_solver", "mumps"},  // 使用MUMPS线性求解器
                     {"ipopt.print_level", 0}});*/
}
