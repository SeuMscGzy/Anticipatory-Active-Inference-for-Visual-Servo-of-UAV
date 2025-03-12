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
  L << -1.8 * poles, 0, 0, -poles * poles, 0, 0, 0, -1.8 * poles, 0, 0,
      -poles * poles, 0, 0, 0, -1.8 * poles, 0, 0, -poles * poles;

  C1.resize(3, 6);
  C1 << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;

  C2.resize(3, 6);
  C2 << 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;
  T_s = T_sampling;
  T_d = T_delay;
  T_f = T_fast;
  T_c = 0.01;
  N_cal = static_cast<int>(T_s / T_c);
  N_p = static_cast<int>(T_s / T_f);
  N_delay = static_cast<int>(T_d / T_c);
  N_minus = N_cal - N_delay;

  yp.resize(N_cal);
  yp_bar.resize(N_cal);
  z_past.resize(N_cal);
  z_future.resize(N_cal);
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
void DR0D::run(Vector3d measure_with_delay) {
  for (int i = 0; i < N_cal; i++) {
    if (i == 0) {
      yp_bar[i] = measure_with_delay;
      yp[i] = yp_bar[i];
      for (int j = N_cal + i - N_delay; j < N_cal; j++) {
        yp[i] += T_c * C2 * z_past[j];
      }
      for (int k = 0; k < i; k++) {
        yp[i] += T_c * C2 * z_future[k];
      }
    } else {
      yp_bar[i] = yp_bar[0];
      if (i <= N_delay) {
        for (int m = N_minus; m < N_minus + i; m++) {
          yp_bar[i] += T_c * C2 * z_past[m];
        }
        yp[i] = yp_bar[i];
        for (int j = N_cal + i - N_delay; j < N_cal; j++) {
          yp[i] += T_c * C2 * z_past[j];
        }
        for (int k = 0; k < i; k++) {
          yp[i] += T_c * C2 * z_future[k];
        }
      } else {
        for (int m = N_minus; m < N_cal; m++) {
          yp_bar[i] += T_c * C2 * z_past[m];
        }
        for (int n = 0; n < i - N_delay; n++) {
          yp_bar[i] += T_c * C2 * z_future[n];
        }
        yp[i] = yp_bar[i];
        for (int j = i - N_delay; j < i; j++) {
          yp[i] += T_c * C2 * z_future[j];
        }
      }
    }
    if (i >= 1) {
      z_future[i] =
          z_future[i - 1] +
          T_c * (A * z_future[i - 1] - L * (yp[i] - C1 * z_future[i - 1]));
    } else {
      z_future[i] =
          z_past[N_cal - 1] +
          T_c * (A * z_past[N_cal - 1] - L * (yp[i] - C1 * z_past[N_cal - 1]));
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
  for (int i = 0; i < N_cal; i++) {
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
  for (int i = 0; i < N_cal; i++) {
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
  for (int i = 0; i < N_cal; i++) {
    z_past[i] = z_temp;
    yp[i] = measure_with_delay;
    yp_bar[i] = measure_with_delay;
    z_future[i] = z_temp;
  }
}