#ifndef MPAI_QPOASES_H
#define MPAI_QPOASES_H

#include <vector>
#include <stdexcept>

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <unsupported/Eigen/KroneckerProduct>

class Optimizer
{
public:
    /**
     * @param dt          采样时间
     * @param Np          预测步长
     * @param precice_z1  位置权重
     * @param precice_z2  速度权重
     * @param precice_w1  参考误差权重（位置）
     * @param precice_w2  参考误差权重（速度）
     * @param precice_z_u 控制增量权重
     * @param e1          预留参数（当前未使用，仅保留接口）
     * @param umin        控制下界
     * @param umax        控制上界
     */
    Optimizer(double dt, int Np,
              double precice_z1, double precice_z2,
              double precice_w1, double precice_w2,
              double precice_z_u,
              double e1,
              double umin, double umax);

    /**
     * @brief 求解一次 MPC
     * @param x          当前位置（3x1）
     * @param vx         当前速度（3x1）
     * @param mu_init    当前 mu（3x1）
     * @param mu_p_init  当前 mu_p（3x1）
     * @return {ux,uy,uz, mu_x_next,mu_y_next,mu_z_next,
     *         mu_p_x_next,mu_p_y_next,mu_p_z_next, cost}
     */
    std::vector<double> optimize(const Eigen::Vector3d &x,
                                 const Eigen::Vector3d &vx,
                                 const Eigen::Vector3d &mu_init,
                                 const Eigen::Vector3d &mu_p_init);

private:
    void initSolver();

    // 参数
    double dt_;
    int    Np_;
    double precice_z1_, precice_z2_;
    double precice_w1_, precice_w2_;
    double precice_z_u_;
    double e1_;
    double umin_, umax_;

    int nV_;   // 决策变量维度 = 3*Np + 6*(Np+1)

    // qpOASES 求解器（只有 bounds 约束）
    qpOASES::QProblemB qpSolver_;

    // 当前状态行向量 [x^T  v^T]
    Eigen::Matrix<double, 1, 6> state_row_;

    // 代价相关大矩阵
    Eigen::MatrixXd    bigMatrix1_;  // Hessian
    Eigen::MatrixXd    bigMatrix2_;  // 6 x nV
    Eigen::RowVectorXd bigMatrix3_;  // 1 x nV

    // 上下界模板和临时变量
    Eigen::VectorXd xLowerTemplate_, xUpperTemplate_;
    Eigen::VectorXd xLowerScratch_, xUpperScratch_;

    // 解与梯度
    Eigen::VectorXd solutionScratch_;
    Eigen::VectorXd q_;  // 列向量形式梯度

    // 行主序 Hessian，给 qpOASES 用
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hessianRowMajor_;
};

#endif // MPAI_QPOASES_H
