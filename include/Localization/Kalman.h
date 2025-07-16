#pragma once

#include "Misc/Matrix.h"
#include <vector>
#include <random>
#include <map>
#include <numeric>
#include <iostream>
#include <cmath>

namespace ExtendedKalmanFilter {
  /* Matrices */
  extern Matrix State;              // // The State Matrix (5x1): [x, y, theta, left_vel, right_vel]

  extern Matrix P;                  // State covariance (5x5)
  extern Matrix Q;                  // Process noise covariance (5x5)
  extern Matrix R;                  // Measurement noise covariance (3x3)
  extern Matrix C;                  // Observation matrix (3x5)

  extern const Matrix& Identity;

  //  Change in Time
  constexpr static double dt = .005;

  // Robot Parameters (set these based on your robot)Simple - Needs to be tuned
  constexpr static double kE = 25; // Adjust based on your motors
  constexpr static double wheel_radius = 1.375;     // Wheel radius in inches
  constexpr static double track_width = 12.5;     // Distance between wheels in inches

  // Helper Functions
  extern Matrix getInput(void);
  extern Matrix getState(void);
  extern Matrix getMeasurement(void);
  
  // EKF Functions
  extern Matrix f(Matrix& x, Matrix& u);
  extern Matrix calculateJacobian(Matrix& x);

  // Initialize Filter
  extern void StartEKF(double init_x = 0, double init_y = 0, double init_theta = 0);

  // Main Loop
  extern void ExtendedKalmanFilter(void);
}