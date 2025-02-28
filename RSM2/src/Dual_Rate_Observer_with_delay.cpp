#include "Dual_Rate_Observer_with_delay.h"

// Constructor to initialize variables
DR0D::DR0D(ros::NodeHandle &nh)
{
    // Initialize matrices and variables
    A << 0, 1, 0, 0;
    L << -20, -100;
    T_s = 0.06;
    T_delay = 0.0;
    T_c = 0.001;
    Np = static_cast<int>(T_s / T_c);
    N_delay = static_cast<int>(T_delay / T_c);
    N_minus = Np - N_delay;

    yp.resize(Np);
    z_past.resize(Np);
    z_future.resize(Np);

    Vector2d z_temp;
    z_temp << 0, 0;
    for (int i = 0; i < Np; i++)
    {
        yp[i] = 0;
        z_past[i] = z_temp;
        z_future[i] = z_temp;
    }
}

// Method to run the DROD calculation loop
void DR0D::run(double measure_with_delay)
{
    // Reset the vectors for the next iteration
    resetVectors();
    z_future[0] = z_past[Np - 1];
    for (int i = 0; i < Np; i++)
    {
        yp[i] = measure_with_delay;
        for (int j = N_minus; j < Np; j++)
        {
            yp[i] += T_c * z_past[j](1);
        }
        for (int k = 0; k < i; k++)
        {
            yp[i] += T_c * z_future[k](1);
        }
        Vector2d z_temp;
        if (i == 0)
        {
            z_temp = z_future[i] + T_c * (A * z_future[i] - L * (yp[i] - z_future[i](0)));
            z_future[i] = z_temp;
        }
        else
        {
            z_temp = z_future[i - 1] + T_c * (A * z_future[i - 1] - L * (yp[i] - z_future[i - 1](0)));
            z_future[i] = z_temp;
        }
    }
    z_past = z_future;
}

// Method to reset vectors for the next iteration
void DR0D::resetVectors()
{
    for (int i = 0; i < Np; i++)
    {
        Vector2d z_temp;
        z_temp << 0, 0;
        yp[i] = 0;
        z_future[i] = z_temp;
    }
}

// Method to reset vectors for the next iteration
void DR0D::resetVectors_past()
{
    for (int i = 0; i < Np; i++)
    {
        Vector2d z_temp;
        z_temp << 0, 0;
        z_past[i] = z_temp;
    }
}
