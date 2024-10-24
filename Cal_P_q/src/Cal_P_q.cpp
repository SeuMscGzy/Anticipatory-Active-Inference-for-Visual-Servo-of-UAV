#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cal_P_q");
    ros::NodeHandle nh;

    // Define the parameters
    double dt = 0.01;
    int Np = 20;
    double precice_z1 = 4;
    double precice_z2 = 0.5;
    double precice_w1 = 2;
    double precice_w2 = 1;
    double precice_z_u = 0.005;
    double e1 = 3;
    double umin = -1;
    double umax = 1;

    // 离散系统矩阵
    MatrixXd Ad(2, 2);
    Ad << 1, dt,
        0, 1;
    MatrixXd Bd(2, 1);
    Bd << 0.5 * dt * dt,
        dt;

    // 构建矩阵 M1 到 M7
    MatrixXd M1(1, 2);
    M1 << e1, 1;

    MatrixXd M2(1, 2);
    M2 << 1, 0;

    MatrixXd M12(1, 2);
    M12 << 0, 1;

    MatrixXd M3(1, 2);
    M3 << 0, e1;

    MatrixXd M4(1, 4);
    M4 << 0, 0, e1, 1;

    MatrixXd M5(1, 4);
    M5 << 1, 0, -1, 0;

    MatrixXd M6(1, 4);
    M6 << 0, 0, 0, e1;

    MatrixXd M7(1, 4);
    M7 << 0, 1, 0, -1;

    MatrixXd M8(4, 4);
    M8 = 0.5 * precice_w1 * M4.transpose() * M4 +
         0.5 * precice_z1 * M5.transpose() * M5 +
         0.5 * precice_w2 * M6.transpose() * M6 +
         0.5 * precice_z2 * M7.transpose() * M7;

    MatrixXd M9 = kroneckerProduct(MatrixXd::Identity(Np + 1, Np + 1), M8);

    int s = 2;
    int num_block_rows = 2 * Np + 2;
    int num_block_cols = 2 * Np + 2;

    MatrixXd M10 = MatrixXd::Zero(s * num_block_rows, s * num_block_cols);
    MatrixXd I = MatrixXd::Identity(s, s);

    // 奇数块
    for (int i_block = 1; i_block <= 2 * Np + 1; i_block += 2)
    {
        int j_block = (i_block + 1) / 2;
        int row_start = (i_block - 1) * s;
        int col_start = (j_block - 1) * s;
        M10.block(row_start, col_start, s, s) = I;
    }

    // 偶数块
    for (int i_block = 2; i_block <= 2 * Np + 2; i_block += 2)
    {
        int j_block = Np + (i_block + 2) / 2;
        int row_start = (i_block - 1) * s;
        int col_start = (j_block - 1) * s;
        M10.block(row_start, col_start, s, s) = I;
    }

    MatrixXd F = MatrixXd::Identity(2, 2);
    MatrixXd F_temp = Ad;
    for (int i = 1; i <= Np; ++i)
    {
        F.conservativeResize(F.rows() + 2, F.cols());
        F.block(2 * i, 0, 2, 2) = F_temp;
        F_temp *= Ad;
    }
    cout << F << endl;
    //  初始化 Phi 矩阵
    MatrixXd Phi((Np + 1) * 2, Np);
    Phi.setZero();

    // 构建 Phi 矩阵
    for (int i = 2; i <= Np + 1; ++i)
    {
        for (int j = 1; j < i; ++j)
        {
            int power = i - j -1;

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
    cout << Phi << endl;

    MatrixXd M11 = MatrixXd::Zero(2 * (Np + 1) * 2, Np + 2 * (Np + 1));
    M11.block(0, 0, 2 * (Np + 1), Np) = Phi;
    M11.block(2 * (Np + 1), Np, 2 * (Np + 1), 2 * (Np + 1)) = MatrixXd::Identity(2 * (Np + 1), 2 * (Np + 1));

    MatrixXd M13 = MatrixXd::Zero(Np + 2 * (Np + 1), Np + 2 * (Np + 1));
    M13.block(0, 0, Np, Np) = MatrixXd::Identity(Np, Np);
    M13.block(Np, Np + 2 * (Np + 1), 2 * (Np + 1), 2 * (Np + 1)) = MatrixXd::Zero(2 * (Np + 1), 2 * (Np + 1));

    MatrixXd bigMatrix1 = 2 * M11.transpose() * M10.transpose() * M9 * M10 * M11 + M13 * precice_z_u;
    //cout << bigMatrix1 << endl;
    //  Compute bigMatrix2 using Eigen
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = precice_z1;
    R(1, 1) = precice_z2;
    Eigen::MatrixXd bigR = Eigen::MatrixXd::Zero(2 * (Np + 1), 2 * (Np + 1));
    for (int i = 0; i < Np + 1; ++i)
    {
        bigR.block(2 * i, 2 * i, 2, 2) = R;
    }

    MatrixXd bigMatrix2 = F.transpose() * bigR;
    return 0;
}