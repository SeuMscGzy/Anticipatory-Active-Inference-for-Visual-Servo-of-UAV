#include "Dual_Rate_Observer_with_delay.h"

// Constructor to initialize variables
DR0D::DR0D(ros::NodeHandle &nh, double T_sampling, double T_delay,
           double T_fast, double poles) {
  // Initialize matrices and variables
  poles = poles;
  A.resize(6, 6);
  A << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  L.resize(6, 3);
  L << -2 * poles, 0, 0, -poles * poles, 0, 0, 0, -2 * poles, 0, 0,
      -poles * poles, 0, 0, 0, -2 * poles, 0, 0, -poles * poles;

  C1.resize(3, 6);
  C1 << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;

  C2.resize(3, 6);
  C2 << 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;
  T_s = T_sampling;
  T_d = T_delay;
  T_f = T_fast;
  T_c = 0.01;
  N_cal = static_cast<int>(T_s / T_c);
  N_p = static_cast<int>((T_s + T_d) / T_f);
  N_delay = static_cast<int>(T_d / T_c);
  N_minus = N_cal - N_delay;

  yp.resize(N_cal + N_delay);
  yp_bar.resize(N_cal + N_delay);
  z_past.resize(N_cal + N_delay);
  z_future.resize(N_cal + N_delay);
  z_future_dt.resize(N_p + 1);

  Vector6d z_temp;
  z_temp << 0, 0, 0, 0, 0, 0;
  Vector3d y_temp;
  y_temp << 0, 0, 0;
  for (int i = 0; i < N_cal; i++) {
    yp[i] = y_temp;
    yp_bar[i] = y_temp;
    z_past[i] = z_temp;
    z_future[i] = z_temp;
  }
  for (int i = 0; i <= N_p; i++) {
    z_future_dt[i] = z_temp;
  }
}

// Method to run the DROD calculation loop
void DR0D::run(const Vector3d &measure_with_delay) {
  for (int i = 0; i < N_cal + N_delay; i++) {
    if (i == 0) {
      yp[i] = measure_with_delay;
      for (int j = N_minus; j < N_cal; j++) {
        yp[0] += T_c * C2 * z_past[j];
      }
      z_future[0] = z_past[N_cal - 1] +
                    T_c * (A * z_past[N_cal - 1] -
                           L * (yp[0] - C1 * z_past[N_cal - 1]));
    } else {
      yp[i] = yp[i - 1] + T_c * C2 * z_future[i - 1];
      z_future[i] =
          z_future[i - 1] +
          T_c * (A * z_future[i - 1] - L * (yp[i] - C1 * z_future[i - 1]));
    }
  }
  int count = static_cast<int>(T_f / T_c);
  for (int i = 0; i <= N_p; i++) {
    if (i == 0) {
      z_future_dt[i] = z_future[count * i + 1];
    } else {
      z_future_dt[i] = z_future[count * i - 1];
    }
  }
  z_past = z_future;
}

// Method to reset vectors for the next iteration
void DR0D::resetVectors() {
  Vector6d z_temp;
  z_temp << 0, 0, 0, 0, 0, 0;
  for (int i = 0; i < N_cal + N_delay; i++) {
    Vector3d y_temp;
    y_temp << 0, 0, 0;
    yp[i] = y_temp;
    yp_bar[i] = y_temp;
    z_future[i] = z_temp;
  }
  for (int i = 0; i <= N_p; i++) {
    z_future_dt[i] = z_temp;
  }
}

// Method to reset vectors for the next iteration
void DR0D::resetVectors_past() {
  for (int i = 0; i < N_cal + N_delay; i++) {
    Vector6d z_temp;
    z_temp << 0, 0, 0, 0, 0, 0;
    z_past[i] = z_temp;
  }
}

void DR0D::init(Vector3d measure_with_delay) {
  Vector6d z_temp;
  z_temp << measure_with_delay(0), 0, measure_with_delay(1), 0,
      measure_with_delay(2), 0;
  z_future_dt[0] = z_temp;
  for (int i = 0; i < N_cal + N_delay; i++) {
    z_past[i] = z_temp;
    yp[i] = measure_with_delay;
    yp_bar[i] = measure_with_delay;
    z_future[i] = z_temp;
  }
}