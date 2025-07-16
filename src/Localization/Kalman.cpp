// Default
#include "vex.h"
#include "cmath"
#include "api.h"

namespace ExtendedKalmanFilter {
  // The State Matrix (5x1): [x, y, theta, left_vel, right_vel]

  Matrix State;              // State estimate (5x1)
  Matrix P;                  // State covariance (5x5)
  Matrix Q;                  // Process noise covariance (5x5)
  Matrix R;                  // Measurement noise covariance (3x3)
  Matrix C;                  // Observation matrix (3x5)

  const Matrix& Identity = Matrix::createIdentity(5, 5);

  Matrix getInput(void);
  Matrix getState(void);
  Matrix getMeasurement(void);
  
  Matrix f(Matrix& x, Matrix& u);
  Matrix calculateJacobian(Matrix& x);

  // Initialize Filter
  void StartEKF(double init_x, double init_y, double init_theta);

  // Main Loop
  void ExtendedKalmanFilter(void);
}

void ExtendedKalmanFilter::StartEKF(double init_x, double init_y, double init_theta) noexcept {
  // Initialize state vector
  State = Matrix(5, 1);
  State(0, 0) = init_x;
  State(1, 0) = init_y;
  State(2, 0) = init_theta;
  State(3, 0) = 0; // Initial left wheel velocity
  State(4, 0) = 0; // Initial right wheel velocity

  // Initialize observation matrix (3x5)
  C = Matrix(3, 5);
  // Only observe x, y, and theta
  C(0, 0) = 1;  // x measurement
  C(1, 1) = 1;  // y measurement
  C(2, 2) = 1;  // theta measurement

  // Initialize covariance matrices
  P = Matrix::createIdentity(5, 5, 0.1); // Initial State uncertainty
  
  // Process noise covariance
  Q = Matrix::createIdentity(5, 5);
  Q(0, 0) = 0.01; // x position noise
  Q(1, 1) = 0.01; // y position noise
  Q(2, 2) = 0.01; // theta noise
  Q(3, 3) = 0.0;  // left velocity noise
  Q(4, 4) = 0.0;  // right velocity noise

  // Measurement noise covariance (3x3)
  R = Matrix(3, 3);
  R(0, 0) = 0.1; // x measurement noise
  R(1, 1) = 0.1; // y measurement noise
  R(2, 2) = 0.05; // theta measurement noise
}

void ExtendedKalmanFilter::ExtendedKalmanFilter(void) {
  while(true) {
    // Update Input
    Matrix u = getInput();

    // Predict state
    State = f(State, u);

    // Calculate Jacobian
    Matrix F = calculateJacobian(State);

    // Predict the Covariances
    P = F * P * F.transpose() + Q;

    // Compute the Kalman Gain
    Matrix K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse();

    // Update Measurement
    Matrix Z = getMeasurement();

    // Update the States
    State = State + K * (Z - C * State);

    // Make the Identity Matrix
    Matrix I = Identity;

    // Update the Covariances
    P = (I - K * C) * P;
    
    // // Wait dt seconds
    wait(dt, sec);
  }
}

// State transition function
Matrix ExtendedKalmanFilter::f(Matrix& x, Matrix& u) noexcept {
  // Next state
  Matrix next_state(5, 1);
  
  // Extract current state values
  const double curr_x = x(0, 0);
  const double curr_y = x(1, 0);
  const double curr_theta = x(2, 0);
  const double curr_left_vel = x(3, 0);
  const double curr_right_vel = x(4, 0);

  // Extract input values (wheel voltages)
  const double left_voltage = u(0, 0);
  const double right_voltage = u(1, 0);

  // Simple motor model (voltage to velocity) // Simplified motor model: acceleration = K * voltage - B * velocity
  constexpr double V = 5.0;
  constexpr double S = 5.0;

  double next_left_vel = curr_left_vel + (V * left_voltage - S * curr_left_vel) * dt;
  double next_right_vel = curr_right_vel + (V * left_voltage - S * curr_left_vel) * dt;

  next_left_vel = fabs(left_voltage) > 1 ? next_left_vel : 0;
  next_right_vel = fabs(right_voltage) > 1 ? next_right_vel : 0;

  // Calculate linear and angular velocities
  const double v = wheel_radius * (next_right_vel + next_left_vel) / 2.0;
  const double omega = wheel_radius * (next_right_vel - next_left_vel) / track_width;

  // Update position and orientation
  next_state(0, 0) = curr_x + v * cos(curr_theta) * dt;
  next_state(1, 0) = curr_y + v * sin(curr_theta) * dt;
  next_state(2, 0) = curr_theta + omega * dt;
  next_state(3, 0) = next_left_vel;
  next_state(4, 0) = next_right_vel;

  // Return Next State
  return next_state;
}

// Jacobian of state transition function
Matrix ExtendedKalmanFilter::calculateJacobian(Matrix& x) {
  // Jacobian Matrix - ∂f(x,u) / ∂x F or A (depending who you ask...) (5x5)
  Matrix F = Matrix(5, 5);
  
  // Extract current state values
  const double theta = x(2,0);
  const double left_vel = x(3,0);
  const double right_vel = x(4,0);

  // Calculate linear and angular velocities
  const double v = wheel_radius * (right_vel + left_vel) / 2.0;
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Fill in the Jacobian matrix
  F = Matrix::createIdentity(5, 5);

  // ∂x/∂theta
  F(0,2) = -v * sin(theta) * dt;
  // ∂x/∂left_vel, ∂x/∂right_vel
  F(0,3) = wheel_radius * cos_theta * dt / 2.0;
  F(0,4) = wheel_radius * cos_theta * dt / 2.0;
  
  // ∂y/∂theta
  F(1,2) = v * cos(theta) * dt;
  // ∂y/∂left_vel, ∂y/∂right_vel
  F(1,3) = wheel_radius * sin_theta * dt / 2.0;
  F(1,4) = wheel_radius * sin_theta * dt / 2.0;

  // Angular velocity partials
  F(2,3) = -wheel_radius * dt / track_width; // ∂theta/∂left_vel
  F(2,4) = wheel_radius * dt / track_width;  // ∂theta/∂right_vel

  return F;
}

Matrix ExtendedKalmanFilter::getMeasurement() {
  return Matrix(vector<vector<double>> {
    {MCL::X},
    {MCL::Y},
    {MCL::theta}
  });
}

Matrix ExtendedKalmanFilter::getInput() {
  // Average the voltage readings from multiple motors
  const double left_voltage = (L1.voltage() + L2.voltage() + L3.voltage()) / 3.0;
  const double right_voltage = (R1.voltage() + R2.voltage() + R3.voltage()) / 3.0;
  
  return Matrix(vector<vector<double>> {
    {left_voltage},
    {right_voltage}
  });
}

Matrix ExtendedKalmanFilter::getState() {
  return State;
}



// // Version 1: Using Torque = Moment of Inertia * Angular Acceleration
// Matrix ExtendedKalmanFilter::f(Matrix& x, Matrix& u) {
//   // Next state
//   Matrix next_state(5, 1);
  
//   // Extract current state values
//   double curr_x = x(0, 0);           // x position in inches
//   double curr_y = x(1, 0);           // y position in inches
//   double curr_theta = x(2, 0);       // orientation in radians
//   double curr_left_vel = x(3, 0);    // left wheel velocity in RPM
//   double curr_right_vel = x(4, 0);   // right wheel velocity in RPM

//   // Extract input values (wheel voltages)
//   double left_voltage = u(0, 0);     // left motor voltage
//   double right_voltage = u(1, 0);    // right motor voltage

//   // Motor parameters
//   constexpr double stall_torque = 2.1;      // N*m (example value - adjust for your motor)
//   constexpr double stall_voltage = 12.0;    // V
//   constexpr double stall_velocity = 630.0;  // RPM
//   constexpr double robot_mass = 10.0;       // kg (adjust based on your robot)
  
//   // Calculate wheel inertia (approximation for a cylinder)
//   double wheel_mass = 0.1;  // kg (adjust for your wheels)
//   double wheel_inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius;  // kg*m²
  
//   // Estimate robot moment of inertia (treating robot as rectangular prism)
//   double robot_width = track_width * 0.0254;  // convert track width to meters
//   double robot_length = 0.5;                  // meters (adjust for your robot)
//   double robot_moment = robot_mass * (robot_width * robot_width + robot_length * robot_length) / 12.0;
  
//   // Total moment of inertia for each side (4 wheels per side + portion of robot mass)
//   double side_inertia = 4 * wheel_inertia + robot_moment / 2.0;  // kg*m²
  
//   // Calculate torque for each side
//   // Convert current velocities from RPM to rad/s
//   double left_vel_rads = curr_left_vel * 2.0 * M_PI / 60.0;
//   double right_vel_rads = curr_right_vel * 2.0 * M_PI / 60.0;
  
//   // Motor torque model: Torque = stall_torque * (voltage/stall_voltage - velocity/stall_velocity)
//   double left_torque = stall_torque * (left_voltage / stall_voltage - left_vel_rads / (stall_velocity * 2.0 * M_PI / 60.0));
//   double right_torque = stall_torque * (right_voltage / stall_voltage - right_vel_rads / (stall_velocity * 2.0 * M_PI / 60.0));
  
//   // Calculate angular acceleration from torque: α = τ / I
//   double left_ang_accel = left_torque / side_inertia;   // rad/s²
//   double right_ang_accel = right_torque / side_inertia; // rad/s²
  
//   // Update velocities using angular acceleration
//   double next_left_vel_rads = left_vel_rads + left_ang_accel * dt;
//   double next_right_vel_rads = right_vel_rads + right_ang_accel * dt;
  
//   // Convert back to RPM
//   double next_left_vel = next_left_vel_rads * 60.0 / (2.0 * M_PI);
//   double next_right_vel = next_right_vel_rads * 60.0 / (2.0 * M_PI);
  
//   // Convert wheel RPM to linear and angular velocities
//   // First convert RPM to rad/s, then to linear velocity in m/s
//   double left_linear_vel = next_left_vel_rads * wheel_radius * 0.0254;   // m/s
//   double right_linear_vel = next_right_vel_rads * wheel_radius * 0.0254; // m/s
  
//   // Calculate robot linear and angular velocities
//   double v = (right_linear_vel + left_linear_vel) / 2.0;                 // m/s
//   double omega = (right_linear_vel - left_linear_vel) / (track_width * 0.0254); // rad/s
  
//   // Update position and orientation
//   // Convert linear velocity from m/s to inches/s
//   double v_inches = v / 0.0254;
  
//   // Update state
//   next_state(0, 0) = curr_x + v_inches * cos(curr_theta) * dt;
//   next_state(1, 0) = curr_y + v_inches * sin(curr_theta) * dt;
//   next_state(2, 0) = curr_theta + omega * dt;
//   next_state(3, 0) = next_left_vel;
//   next_state(4, 0) = next_right_vel;
  
//   return next_state;
// }

// // Version 2: Using Torque = Force * Radius and F = ma
// Matrix ExtendedKalmanFilter::f(Matrix& x, Matrix& u) {
//   // Next state
//   Matrix next_state(5, 1);
  
//   // Extract current state values
//   double curr_x = x(0, 0);           // x position in inches
//   double curr_y = x(1, 0);           // y position in inches
//   double curr_theta = x(2, 0);       // orientation in radians
//   double curr_left_vel = x(3, 0);    // left wheel velocity in RPM
//   double curr_right_vel = x(4, 0);   // right wheel velocity in RPM

//   // Extract input values (wheel voltages)
//   double left_voltage = u(0, 0);     // left motor voltage
//   double right_voltage = u(1, 0);    // right motor voltage

//   // Motor parameters
//   constexpr double stall_torque = 2.1;      // N*m (example value - adjust for your motor)
//   constexpr double stall_voltage = 12.0;    // V
//   constexpr double stall_velocity = 630.0;  // RPM
  
//   // Robot parameters
//   constexpr double robot_mass = 10.0;       // kg (adjust based on your robot)
//   constexpr double wheel_mass = 0.1;        // kg (adjust for your wheels)
  
//   // Mass for each side (half robot mass + 4 wheels)
//   double side_mass = robot_mass / 2.0 + 4.0 * wheel_mass;  // kg
  
//   // Convert current velocities from RPM to rad/s
//   double left_vel_rads = curr_left_vel * 2.0 * M_PI / 60.0;
//   double right_vel_rads = curr_right_vel * 2.0 * M_PI / 60.0;
  
//   // Motor torque model: Torque = stall_torque * (voltage/stall_voltage - velocity/stall_velocity)
//   double left_torque = stall_torque * (left_voltage / stall_voltage - left_vel_rads / (stall_velocity * 2.0 * M_PI / 60.0));
//   double right_torque = stall_torque * (right_voltage / stall_voltage - right_vel_rads / (stall_velocity * 2.0 * M_PI / 60.0));
  
//   // Convert wheel radius to meters
//   double wheel_radius_m = wheel_radius * 0.0254;  // meters
  
//   // Calculate force at wheel contact point: F = τ / r
//   double left_force = left_torque / wheel_radius_m;   // N
//   double right_force = right_torque / wheel_radius_m; // N
  
//   // Calculate linear acceleration: a = F / m
//   double left_accel = left_force / side_mass;   // m/s²
//   double right_accel = right_force / side_mass; // m/s²
  
//   // Update wheel velocities
//   // First convert wheel RPM to linear velocity in m/s
//   double left_linear_vel = left_vel_rads * wheel_radius_m;
//   double right_linear_vel = right_vel_rads * wheel_radius_m;
  
//   // Update linear velocities with acceleration
//   double next_left_linear_vel = left_linear_vel + left_accel * dt;
//   double next_right_linear_vel = right_linear_vel + right_accel * dt;
  
//   // Convert back to angular velocity in rad/s
//   double next_left_vel_rads = next_left_linear_vel / wheel_radius_m;
//   double next_right_vel_rads = next_right_linear_vel / wheel_radius_m;
  
//   // Convert to RPM
//   double next_left_vel = next_left_vel_rads * 60.0 / (2.0 * M_PI);
//   double next_right_vel = next_right_vel_rads * 60.0 / (2.0 * M_PI);
  
//   // Calculate robot linear and angular velocities
//   double v = (next_right_linear_vel + next_left_linear_vel) / 2.0;  // m/s
//   double omega = (next_right_linear_vel - next_left_linear_vel) / (track_width * 0.0254);  // rad/s
  
//   // Convert linear velocity from m/s to inches/s
//   double v_inches = v / 0.0254;
  
//   // Update state
//   next_state(0, 0) = curr_x + v_inches * cos(curr_theta) * dt;
//   next_state(1, 0) = curr_y + v_inches * sin(curr_theta) * dt;
//   next_state(2, 0) = curr_theta + omega * dt;
//   next_state(3, 0) = next_left_vel;
//   next_state(4, 0) = next_right_vel;
  
//   return next_state;
// }



// // Jacobian for Version 1: Using Torque = Moment of Inertia * Angular Acceleration
// Matrix ExtendedKalmanFilter::calculateJacobian(Matrix& x) {
//   // Jacobian Matrix - ∂f(x,u) / ∂x (5x5)
//   Matrix F(5, 5);
  
//   // Extract current state values
//   double theta = x(2, 0);           // orientation in radians
//   double left_vel = x(3, 0);        // left wheel velocity in RPM
//   double right_vel = x(4, 0);       // right wheel velocity in RPM

//   // Convert RPM to rad/s
//   double left_vel_rads = left_vel * 2.0 * M_PI / 60.0;
//   double right_vel_rads = right_vel * 2.0 * M_PI / 60.0;
  
//   // Convert wheel radius to meters
//   double wheel_radius_m = wheel_radius * 0.0254;  // meters
//   double track_width_m = track_width * 0.0254;    // meters
  
//   // Calculate linear velocities
//   double left_linear_vel = left_vel_rads * wheel_radius_m;   // m/s
//   double right_linear_vel = right_vel_rads * wheel_radius_m; // m/s
  
//   // Calculate robot linear and angular velocities
//   double v = (right_linear_vel + left_linear_vel) / 2.0;     // m/s
  
//   // Convert to inches/s
//   double v_inches = v / 0.0254;
  
//   // Start with identity matrix
//   F = Matrix::createIdentity(5, 5);
  
//   // Partial derivatives for position update
//   F(0, 2) = -v_inches * sin(theta) * dt;
//   F(0, 3) = wheel_radius * cos(theta) * dt * (M_PI / 30.0);  // Convert from RPM
//   F(0, 4) = wheel_radius * cos(theta) * dt * (M_PI / 30.0);  // Convert from RPM
  
//   F(1, 2) = v_inches * cos(theta) * dt;
//   F(1, 3) = wheel_radius * sin(theta) * dt * (M_PI / 30.0);  // Convert from RPM
//   F(1, 4) = wheel_radius * sin(theta) * dt * (M_PI / 30.0);  // Convert from RPM
  
//   // Partial derivatives for orientation update
//   F(2, 3) = -wheel_radius * dt / track_width * (M_PI / 30.0);  // Convert from RPM
//   F(2, 4) = wheel_radius * dt / track_width * (M_PI / 30.0);   // Convert from RPM
  
//   // Motor parameters
//   constexpr double stall_torque = 2.1;      // N*m (example value - adjust for your motor)
//   constexpr double stall_voltage = 12.0;    // V
//   constexpr double stall_velocity = 630.0;  // RPM
  
//   // Calculate stall velocity in rad/s
//   double stall_vel_rads = stall_velocity * 2.0 * M_PI / 60.0;
  
//   // Partial derivatives for velocity update
//   // ∂(next_left_vel)/∂(left_vel) and ∂(next_right_vel)/∂(right_vel)
//   F(3, 3) = 1.0 - (stall_torque / stall_vel_rads) * dt / side_inertia * (2.0 * M_PI / 60.0);
//   F(4, 4) = 1.0 - (stall_torque / stall_vel_rads) * dt / side_inertia * (2.0 * M_PI / 60.0);
  
//   return F;
// }

// // Jacobian for Version 2: Using Torque = Force * Radius and F = ma
// Matrix ExtendedKalmanFilter::calculateJacobian(Matrix& x) {
//   // Jacobian Matrix - ∂f(x,u) / ∂x (5x5)
//   Matrix F(5, 5);
  
//   // Extract current state values
//   double theta = x(2, 0);           // orientation in radians
//   double left_vel = x(3, 0);        // left wheel velocity in RPM
//   double right_vel = x(4, 0);       // right wheel velocity in RPM

//   // Convert RPM to rad/s
//   double left_vel_rads = left_vel * 2.0 * M_PI / 60.0;
//   double right_vel_rads = right_vel * 2.0 * M_PI / 60.0;
  
//   // Convert wheel radius to meters
//   double wheel_radius_m = wheel_radius * 0.0254;  // meters
//   double track_width_m = track_width * 0.0254;    // meters
  
//   // Calculate linear velocities
//   double left_linear_vel = left_vel_rads * wheel_radius_m;   // m/s
//   double right_linear_vel = right_vel_rads * wheel_radius_m; // m/s
  
//   // Calculate robot linear and angular velocities
//   double v = (right_linear_vel + left_linear_vel) / 2.0;     // m/s
  
//   // Convert to inches/s
//   double v_inches = v / 0.0254;
  
//   // Start with identity matrix
//   F = Matrix::createIdentity(5, 5);
  
//   // Partial derivatives for position update
//   F(0, 2) = -v_inches * sin(theta) * dt;
//   F(0, 3) = wheel_radius * cos(theta) * dt * (M_PI / 30.0);  // Convert from RPM
//   F(0, 4) = wheel_radius * cos(theta) * dt * (M_PI / 30.0);  // Convert from RPM
  
//   F(1, 2) = v_inches * cos(theta) * dt;
//   F(1, 3) = wheel_radius * sin(theta) * dt * (M_PI / 30.0);  // Convert from RPM
//   F(1, 4) = wheel_radius * sin(theta) * dt * (M_PI / 30.0);  // Convert from RPM
  
//   // Partial derivatives for orientation update
//   F(2, 3) = -wheel_radius * dt / track_width * (M_PI / 30.0);  // Convert from RPM
//   F(2, 4) = wheel_radius * dt / track_width * (M_PI / 30.0);   // Convert from RPM
  
//   // Motor parameters
//   constexpr double stall_torque = 2.1;      // N*m (example value - adjust for your motor)
//   constexpr double stall_voltage = 12.0;    // V
//   constexpr double stall_velocity = 630.0;  // RPM
//   constexpr double robot_mass = 10.0;       // kg
//   constexpr double wheel_mass = 0.1;        // kg
  
//   // Mass for each side (half robot mass + 4 wheels)
//   double side_mass = robot_mass / 2.0 + 4.0 * wheel_mass;  // kg
  
//   // Calculate stall velocity in rad/s
//   double stall_vel_rads = stall_velocity * 2.0 * M_PI / 60.0;
  
//   // Partial derivatives for velocity update
//   // ∂(next_left_vel)/∂(left_vel) and ∂(next_right_vel)/∂(right_vel)
//   F(3, 3) = 1.0 - (stall_torque / (wheel_radius_m * side_mass * stall_vel_rads)) * dt * (2.0 * M_PI / 60.0);
//   F(4, 4) = 1.0 - (stall_torque / (wheel_radius_m * side_mass * stall_vel_rads)) * dt * (2.0 * M_PI / 60.0);
  
//   return F;
// }






























































/* OLD ONE */
// Matrix ExtendedKalmanFilter::f(Matrix& x, Matrix& u) {
//   constexpr double wheel_radius = 2.75 / 2;
//   constexpr double stall_torque = .47;
//   constexpr double stall_velocity_rpm = 625;
//   constexpr double mass_kg = 6.5;
//   constexpr double track_width = 12.5;
//   Matrix next_state(5, 1);
  
//   double curr_x = x(0, 0);
//   double curr_y = x(1, 0);
//   double curr_theta = x(2, 0);
//   double curr_left_vel = x(3, 0);
//   double curr_right_vel = x(4, 0);

//   double left_voltage = u(0, 0);
//   double right_voltage = u(1, 0);

//   // Convert wheel radius to meters for torque calculations
//   double wheel_radius_m = wheel_radius * 0.0254;

//   // Convert current velocities to RPM
//   double left_rpm = (curr_left_vel / wheel_radius) * (60.0 / (2 * M_PI));
//   double right_rpm = (curr_right_vel / wheel_radius) * (60.0 / (2 * M_PI));

//   // Calculate motor torques
//   double torque_left = stall_torque * (left_voltage / 12 - left_rpm / stall_velocity_rpm);
//   double torque_right = stall_torque * (right_voltage / 12 - right_rpm / stall_velocity_rpm);



//   // Convert torque to force (Newtons)
//   double F_left = torque_left / wheel_radius_m;
//   double F_right = torque_right / wheel_radius_m;

//   // Compute accelerations (m/s²)
//   double a_left = F_left / mass_kg;
//   double a_right = F_right / mass_kg;

//   // Convert accelerations to inches/s²
//   double a_left_inch = a_left * 39.3701;
//   double a_right_inch = a_right * 39.3701;



//   // Update velocities
//   double next_left_vel = fabs(left_voltage) > 1 ? curr_left_vel + a_left_inch * dt : 0;
//   double next_right_vel = fabs(right_voltage) > 1 ? curr_right_vel + a_right_inch * dt: 0;

//   // Calculate linear and angular velocities
//   double v = (next_left_vel + next_right_vel) / 2.0;
//   double omega = (next_left_vel - next_right_vel) / track_width;

//   // Update position and orientation
//   next_state(0, 0) = curr_x + v * cos(curr_theta) * dt;
//   next_state(1, 0) = curr_y + v * sin(curr_theta) * dt;
//   next_state(2, 0) = curr_theta + omega * dt;

//   next_state(3, 0) = next_left_vel;
//   next_state(4, 0) = next_right_vel;

//   return next_state;
// }




namespace UnscentedKalmanFilter {
  // The State Matrix (5x1): [x, y, theta, left_vel, right_vel]
  Matrix State;              // State estimate (5x1)
  Matrix P;                  // State covariance (5x5)
  Matrix C;                  // Observation matrix (3x5)

  // Define the state and input dimensions
  const int STATE_DIM = 5;
  const int MEAS_DIM = 3;

  // Define the process noise and measurement noise covariance matrices
  Matrix Q(STATE_DIM, STATE_DIM); // Process noise covariance (5x5)
  Matrix R(MEAS_DIM, MEAS_DIM); // Measurement noise covariance (3x3)

   // Robot physical parameters
  // const double wheel_radius = 0.051;  // meters
  // const double track_width = 0.32;    // meters between wheels
  // const double dt = 0.005;             // 5ms control loop
    
  // Weight calculations for UKF
  vector<double> Wm;  // Weights for mean
  vector<double> Wc;  // Weights for covariance

  // Define the UKF parameters
  const double alpha = 1e-3;
  const double beta = 2;
  const double kappa = 0;
  const double lambda = alpha * alpha * (STATE_DIM + kappa) - STATE_DIM;
  const int sigma_num = 2 * STATE_DIM + 1;

  Matrix getInput(void);
  Matrix getState(void);
  Matrix getMeasurement(void);
  void StartUKF(double init_x, double init_y, double init_theta);
  
  Matrix f(Matrix& x, Matrix& u);

  vector<Matrix> generateSigmaPoints(Matrix& State, Matrix& P);
  // Main Loop
  void UnscentedKalmanFilter(void);

}

void UnscentedKalmanFilter::StartUKF(double init_x, double init_y, double init_theta) {
  // Initialize state vector
  State = Matrix(5, 1);
  State(0, 0) = init_x;
  State(1, 0) = init_y;
  State(2, 0) = init_theta;
  State(3, 0) = 0; // Initial left wheel velocity
  State(4, 0) = 0; // Initial right wheel velocity

  // Initialize observation matrix (3x5)
  C = Matrix(3, 5);
  // Only observe x, y, and theta
  C(0, 0) = 1;  // x measurement
  C(1, 1) = 1;  // y measurement
  C(2, 2) = 1;  // theta measurement

  // Initialize covariance matrices
  P = Matrix::createIdentity(5, 5, 0.1); // Initial State uncertainty
  
  // Process noise covariance
  Q = Matrix::createIdentity(5, 5);
  Q(0, 0) = 0.01; // x position noise
  Q(1, 1) = 0.01; // y position noise
  Q(2, 2) = 0.01; // theta noise
  Q(3, 3) = 0.1;  // left velocity noise
  Q(4, 4) = 0.1;  // right velocity noise

  // Measurement noise covariance (3x3)
  R = Matrix(3, 3);
  R(0, 0) = 0.1; // x measurement noise
  R(1, 1) = 0.1; // y measurement noise
  R(2, 2) = 0.05; // theta measurement noise

  // Calculate UKF parameters
  
  // Initialize weights
  Wm.resize(sigma_num);
  Wc.resize(sigma_num);
  
  Wm[0] = lambda / (STATE_DIM + lambda);
  Wc[0] = Wm[0] + (1 - alpha * alpha + beta);
  
  for (int i = 1; i < sigma_num; i++) {
    Wc[i] = Wm[i] = 1.0 / (2.0 * (STATE_DIM + lambda));
  }
}

// Ensures matrix symmetry by averaging with its transpose
// This helps prevent numerical issues in the Cholesky decomposition
inline Matrix enforceSymmetry(Matrix& A) {
  return (A + A.transpose()) * 0.5;
}

void UnscentedKalmanFilter::UnscentedKalmanFilter(void) {
  while(true) {
    Matrix u = getInput();
    // Generate sigma points
    vector<Matrix> sigmaPoints = generateSigmaPoints(State, P);
    
    // Predict sigma points
    vector<Matrix> predictedSigma(sigma_num, Matrix(STATE_DIM, 1));
    for (int i = 0; i < sigma_num; ++i) {
      predictedSigma[i] = f(sigmaPoints[i], u);
    }
    
    // Predict mean
    Matrix xPred = Matrix(STATE_DIM, 1); 
    for (int i = 0; i < sigma_num; ++i) {
      xPred += predictedSigma[i] * Wm[i];
    }
    
    // Predict covariance with enforced symmetry
    Matrix PPred = Q;  // Start with process noise
    for (int i = 0; i < sigma_num; ++i) {
      Matrix diff = predictedSigma[i] - xPred;
      PPred += (diff * diff.transpose()) * Wc[i];
    }

    PPred = enforceSymmetry(PPred);  // Ensure symmetry
    
    // Update state and covariance
    State = xPred;
    P = PPred;

    // Get Measurement
    Matrix Z = getMeasurement();

    // Transform sigma points to measurement space
    vector<Matrix> measurementSigmaPoints(sigma_num, Matrix(MEAS_DIM, 1));
    for (int i = 0; i < sigma_num; i++) {
      measurementSigmaPoints[i] = C * sigmaPoints[i];
    }

    // Calculate mean predicted measurement
    Matrix zPred = Matrix(MEAS_DIM, 1, 0.0);
    for (int i = 0; i < sigma_num; i++) {
      zPred = zPred + measurementSigmaPoints[i] * Wm[i];
    }

    // Calculate innovation covariance
    Matrix S = R;
    for (int i = 0; i < sigma_num; i++) {
      Matrix diff = measurementSigmaPoints[i] - zPred;
      S = S + (diff * diff.transpose()) * Wc[i];
    }
    // Keeps it Symetrical
    S = enforceSymmetry(S);
    
    // Calculate cross-covariance
    Matrix Pxz = Matrix(STATE_DIM, MEAS_DIM, 0.0);
    for (int i = 0; i < sigma_num; i++) {
      Matrix diffX = sigmaPoints[i] - State;
      Matrix diffZ = measurementSigmaPoints[i] - zPred;
      Pxz = Pxz + (diffX * diffZ.transpose()) * Wc[i];
    }
    
    // Calculate Kalman gain
    Matrix K = Pxz * S.inverse();
    
    // Update state and covariance
    State += K * (Z - zPred);
    P -= K * S * K.transpose();

    P = enforceSymmetry(P);
    wait(5, msec);
  }
}

vector<Matrix> UnscentedKalmanFilter::generateSigmaPoints(Matrix& State, Matrix& P) {
  // Vector of Matrices
  vector<Matrix> sigmaPoints(sigma_num, Matrix(STATE_DIM, 1));
  
  // Calculate scaled covariance matrix
  Matrix scaledP = P * (STATE_DIM + lambda);
  
  // Compute Cholesky decomposition
  Matrix L = scaledP.cholesky();
  
  // Set mean as first sigma point
  sigmaPoints[0] = State;
  
  // Generate remaining sigma points using the Cholesky factor
  for (int i = 0; i < STATE_DIM; i++) {
    Matrix columnVector(STATE_DIM, 1);

    for (int j = 0; j < STATE_DIM; j++) {
      columnVector(j,0) = L(j,i);
    }
      
    sigmaPoints[i + 1] = State + columnVector;
    sigmaPoints[i + STATE_DIM + 1] = State - columnVector;
  }
  
  return sigmaPoints;
}

// State transition function
Matrix UnscentedKalmanFilter::f(Matrix& x, Matrix& u) noexcept {
  Matrix next_state(5, 1);
  
  // // Extract current state values
  // static const double curr_x = x(0, 0);        // x position (inches)
  // double curr_y = x(1, 0);        // y position (inches)
  // double curr_theta = x(2, 0);    // orientation (radians)
  // double curr_left_vel = x(3, 0); // left wheel velocity (inches/s)
  // double curr_right_vel = x(4, 0);// right wheel velocity (inches/s)

  // // Extract input values (wheel voltages)
  // double left_voltage = u(0, 0);   // left motor voltage
  // double right_voltage = u(1, 0);  // right motor voltage

  // // Convert current velocities to m/s for calculations
  // double left_vel_ms = curr_left_vel * 0.0254;
  // double right_vel_ms = curr_right_vel * 0.0254;

  // // Calculate torque for each wheel using the provided equation
  // double left_torque = stall_torque * (left_voltage / stall_voltage - left_vel_ms / (wheel_radius_m * free_speed_rads));
  
  // double right_torque = stall_torque * (right_voltage / stall_voltage - right_vel_ms / (wheel_radius_m * free_speed_rads));

  // // Calculate accelerations
  // double left_accel_ms2 = left_torque / (wheel_mass_kg * wheel_radius_m);
  // double right_accel_ms2 = right_torque / (wheel_mass_kg * wheel_radius_m);

  // // Convert accelerations to in/s²
  // double left_accel = left_accel_ms2 * 39.3701;
  // double right_accel = right_accel_ms2 * 39.3701;

  // // Update velocities
  // double next_left_vel = curr_left_vel + left_accel * dt;
  // double next_right_vel = curr_right_vel + right_accel * dt;

  // // Calculate linear and angular velocities
  // double v = wheel_radius_in * (next_right_vel + next_left_vel) / 2.0;
  // double omega = wheel_radius_in * (next_right_vel - next_left_vel) / track_width_in;

  // // Update position and orientation
  // next_state(0, 0) = curr_x + v * cos(curr_theta) * dt;
  // next_state(1, 0) = curr_y + v * sin(curr_theta) * dt;
  // next_state(2, 0) = curr_theta + omega * dt;
  // next_state(3, 0) = next_left_vel;
  // next_state(4, 0) = next_right_vel;

  return next_state;
}


Matrix UnscentedKalmanFilter::getMeasurement(void) noexcept { // 3 x 1
  return Matrix(vector<vector<double>> {{MCL::X}, {MCL::Y}, {MCL::theta}});
}

Matrix UnscentedKalmanFilter::getInput(void) noexcept {
  return Matrix(vector<vector<double>> {
    {(L1.voltage() + L2.voltage() + L3.voltage()) * .3333333},
    {(R1.voltage() + R2.voltage() + R3.voltage()) * .3333333}
  });
}

Matrix UnscentedKalmanFilter::getState(void) noexcept {
  return State;
}