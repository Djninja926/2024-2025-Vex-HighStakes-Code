// // Default
// #include "vex.h"
// #include "cmath"
// #include "api.h"

// using namespace std;

// namespace MPC {
//   // --- Robot Physical Parameters ---
//   const double WHEEL_DIAMETER_IN = 3.25; // Inches
//   const double WHEEL_CIRCUMFERENCE_IN = M_PI * WHEEL_DIAMETER_IN;
//   const double TRACK_WIDTH_IN = 13.5; // Inches - Distance between wheels

//   // --- Motor Model Parameters (NEEDS TUNING) ---
//   // Simplified model: v_next = v_current + k_response * (k_volt_to_rpm * voltage - v_current) * dt
//   // These are highly dependent on gearing, motor type (V5 Smart Motor), and load
//   const double K_VOLTS_TO_RPM = 18.0; // RPM per Volt at free speed (approx for 200 RPM Cartridge) - TUNE THIS
//   const double K_MOTOR_RESPONSE = 15.0; // How quickly motor reaches target speed (unitless factor) - TUNE THIS
//   const double MAX_VOLTAGE = 12.0;    // Volts
//   const double MAX_RPM = 200.0;       // Physical max RPM of the motors used

//   // --- MPC Configuration ---
//   const int PREDICTION_HORIZON = 20;      // N: Number of steps to predict ahead
//   const double TIMESTEP_DT = 0.02;        // dt: Control loop frequency (seconds) -> 50 Hz
//   const int OPTIMIZATION_ITERATIONS = 5;  // Number of gradient descent iterations per timestep
//   const double LEARNING_RATE = 0.1;       // Gradient descent learning rate - TUNE THIS
//   const double FINITE_DIFF_EPSILON = 0.01; // Small change for numerical gradient calculation

//   // --- Cost Function Weights (NEEDS TUNING) ---
//   // State weighting matrix Q (diagonal)
//   const double Q_X = 10.0;        // Weight for X error
//   const double Q_Y = 10.0;        // Weight for Y error
//   const double Q_THETA = 5.0;     // Weight for Theta error
//   const double Q_VL = 0.5;        // Weight for Left Velocity error
//   const double Q_VR = 0.5;        // Weight for Right Velocity error

//   // Input weighting matrix R (diagonal)
//   const double R_VL = 0.1;         // Penalty for Left Voltage magnitude
//   const double R_VR = 0.1;         // Penalty for Right Voltage magnitude

//   // Input rate-of-change weighting matrix S (diagonal)
//   const double S_VL_DOT = 0.05;     // Penalty for Left Voltage change (acceleration)
//   const double S_VR_DOT = 0.05;     // Penalty for Right Voltage change (acceleration)

//   // Helper to create diagonal Q matrix
//   Matrix createQMatrix(void) {
//     Matrix Q = Matrix(5, 5);
//     Q(0, 0) = Q_X;
//     Q(1, 1) = Q_Y;
//     Q(2, 2) = Q_THETA;
//     Q(3, 3) = Q_VL;
//     Q(4, 4) = Q_VR;
//     return Q;
//   }

//   // Helper to create diagonal R matrix
//   Matrix createRMatrix(void) {
//     Matrix R = Matrix(2, 2);
//     R(0, 0) = R_VL;
//     R(1, 1) = R_VR;
//     return R;
//   }

//   // Helper to create diagonal S matrix
//   Matrix createSMatrix(void) {
//     Matrix S = Matrix(2, 2);
//     S(0, 0) = S_VL_DOT;
//     S(1, 1) = S_VR_DOT;
//     return S;
//   }

//   // Define state and input vector sizes
//   const int STATE_DIM = 5; // [X, Y, Theta, vL, vR]
//   const int INPUT_DIM = 2; // [VoltL, VoltR]

//   // Type aliases for clarity
//   Matrix State(STATE_DIM, 1); // Expected to be 5x1
//   Matrix Input(INPUT_DIM, 1); // Expected to be 2x1



//   // Converts RPM to inches per second
//   double rpmToInchesPerSec(double rpm) {
//     return rpm * WHEEL_CIRCUMFERENCE_IN / 60.0;
//   }

//   // Converts inches per second to RPM
//   double inchesPerSecToRpm(double ips) {
//     return ips * 60.0 / WHEEL_CIRCUMFERENCE_IN;
//   }


//   /**
//  * Predicts the next state of the robot given the current state and input voltages.
//  * @param current_state Matrix [X, Y, Theta, vL_rpm, vR_rpm] (5x1)
//  * @param input Matrix [VoltL, VoltR] (2x1)
//  * @param dt Time step (seconds)
//  * @return Predicted next Matrix (5x1)
//  */
//   Matrix predict_next_state(Matrix& current_state, Matrix& input, double dt) {
//     double x = current_state(0, 0);
//     double y = current_state(1, 0);
//     double theta = current_state(2, 0);
//     double vL_rpm = current_state(3, 0);
//     double vR_rpm = current_state(4, 0);

//     double voltL = input(0, 0);
//     double voltR = input(1, 0);

//     // --- Dynamics Update (Velocity) ---
//     // Simplified motor model: v_next = v + k_response * (v_target - v) * dt
//     // Where v_target is roughly proportional to voltage
//     double target_vL_rpm = K_VOLTS_TO_RPM * voltL;
//     double target_vR_rpm = K_VOLTS_TO_RPM * voltR;

//     // Clamp target RPM to physical limits if needed, though voltage limit should handle this
//     // target_vL_rpm = std::max(-MAX_RPM, std::min(MAX_RPM, target_vL_rpm));
//     // target_vR_rpm = std::max(-MAX_RPM, std::min(MAX_RPM, target_vR_rpm));

//     double next_vL_rpm = vL_rpm + K_MOTOR_RESPONSE * (target_vL_rpm - vL_rpm) * dt;
//     double next_vR_rpm = vR_rpm + K_MOTOR_RESPONSE * (target_vR_rpm - vR_rpm) * dt;

//     // Clamp resulting RPM if model predicts unrealistic values
//     next_vL_rpm = std::max(-MAX_RPM*1.1, std::min(MAX_RPM*1.1, next_vL_rpm)); // Allow slight overshoot
//     next_vR_rpm = std::max(-MAX_RPM*1.1, std::min(MAX_RPM*1.1, next_vR_rpm));

//     // --- Kinematics Update (Position) ---
//     double vL_ips = rpmToInchesPerSec(vL_rpm); // Use current velocity for kinematics calc
//     double vR_ips = rpmToInchesPerSec(vR_rpm);

//     double linear_velocity_ips = (vL_ips + vR_ips) / 2.0;
//     double angular_velocity_rps = (vR_ips - vL_ips) / TRACK_WIDTH_IN; // Radians per second

//     double next_x = x + linear_velocity_ips * cos(theta) * dt;
//     double next_y = y + linear_velocity_ips * sin(theta) * dt;
//     double next_theta = theta + angular_velocity_rps * dt;
//     next_theta = Normalize(next_theta); // Keep theta within [-pi, pi]

//     // --- Create Next State Vector ---
//     Matrix next_state(STATE_DIM, 1);
//     next_state(0, 0) = next_x;
//     next_state(1, 0) = next_y;
//     next_state(2, 0) = next_theta;
//     next_state(3, 0) = next_vL_rpm;
//     next_state(4, 0) = next_vR_rpm;

//     return next_state;
//   }


//   // Pre-calculated weight matrices
//   Matrix Q;
//   Matrix R;
//   Matrix S;

//   // Reference trajectory generated by ProfileGenerator
//   vector<ProfilePoint> full_trajectory_profile;
//   double trajectory_dd; // Step size used in profile generation

//   // Current optimal input sequence (shifted and reused)
//   vector<Matrix> current_input_sequence;

//   // MPCController(const vector<ProfilePoint>& trajectory, double dd) : full_trajectory_profile(trajectory), trajectory_dd(dd) {
//   //   Q = createQMatrix();
//   //   R = createRMatrix();
//   //   S = createSMatrix();

//   //   // Initialize input sequence with zeros
//   //   current_input_sequence.resize(PREDICTION_HORIZON, Matrix(INPUT_DIM, 1));
//   // }


//   /**
//   * Finds the index of the closest point on the reference trajectory.
//   * Simple implementation: assumes progress mostly forward. More robust needed for complex paths.
//   */
//   int findClosestTrajectoryPoint(Matrix& current_state) {
//     double current_x = current_state(0, 0);
//     double current_y = current_state(1, 0);
//     double min_dist_sq = -1.0;
//     int closest_idx = 0;
//     // Simple search - assumes we are generally moving forward
//     // A better search might start near the last known closest index
//     for (int i = 0; i < full_trajectory_profile.size(); ++i) {
//       double dx = full_trajectory_profile[i].x - current_x;
//       double dy = full_trajectory_profile[i].y - current_y;
//       double dist_sq = dx * dx + dy * dy;
//       if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
//         min_dist_sq = dist_sq;
//         closest_idx = i;
//       }
//     }
//     return closest_idx;
//   }

//    /**
//     * Extracts the reference state sequence for the prediction horizon.
//     * Starts from the closest point on the generated profile.
//     * Needs interpolation if TIMESTEP_DT doesn't align with profile generation step 'dd'.
//     * Simplified: picks nearest points based on estimated time/distance.
//     */
//   vector<Matrix> getReferenceSequence(int start_index, const Matrix& current_state) {
//     vector<Matrix> ref_sequence;
//     ref_sequence.reserve(PREDICTION_HORIZON);

//     double current_dist_on_traj = full_trajectory_profile[start_index].dist;
//     double current_vel_ref = full_trajectory_profile[start_index].vel; // Inches per sec
//     for (int k = 1; k <= PREDICTION_HORIZON; ++k) {
//       // Estimate distance travelled in k steps based on reference speed at start
//       // More accurate: use predicted speeds, but this is simpler for reference lookup
//       double estimated_future_dist = current_dist_on_traj + current_vel_ref * (k * TIMESTEP_DT);

//       // Find the index in the profile corresponding to this estimated distance
//       // (This assumes profile points are closely spaced via 'dd')
//       int ref_idx = static_cast<int>(round(estimated_future_dist / trajectory_dd));
//       ref_idx = std::max(0, std::min((int)full_trajectory_profile.size() - 1, ref_idx));

//       const auto& ref_point = full_trajectory_profile[ref_idx];

//       Matrix ref_state(STATE_DIM, 1);
//       ref_state(0, 0) = ref_point.x;
//       ref_state(1, 0) = ref_point.y;
//       ref_state(2, 0) = Normalize(ref_point.theta); // Ensure normalized

//       // Calculate target wheel velocities (RPM) from linear/angular velocities
//       // Use the inverse kinematics logic from Constraints::wheelSpeeds
//       double target_lin_vel_ips = ref_point.vel;
//       double target_ang_vel_rps = ref_point.omega; // Assuming ProfilePoint stores omega in rad/s

//       double v_right_ips = target_lin_vel_ips + target_ang_vel_rps * TRACK_WIDTH_IN / 2.0;
//       double v_left_ips  = target_lin_vel_ips - target_ang_vel_rps * TRACK_WIDTH_IN / 2.0;

//       ref_state(3, 0) = inchesPerSecToRpm(v_left_ips);
//       ref_state(4, 0) = inchesPerSecToRpm(v_right_ips);

//       ref_sequence.push_back(ref_state);
//     }
//     return ref_sequence;
//   }


//   /**
//   * Calculates the total cost for a predicted trajectory and input sequence.
//   */
//   double calculate_cost(vector<Matrix>& predicted_states, vector<Matrix>& inputs, vector<Matrix>& reference_states) {
//     double total_cost = 0.0;
//     Matrix prev_input = current_input_sequence[0]; // Use last applied input approx
//     for (int k = 0; k < PREDICTION_HORIZON; ++k) {
//         // State Error Cost (x - x_ref)^T * Q * (x - x_ref)
//         Matrix state_error = predicted_states[k] - reference_states[k];
//         // Ensure theta error is handled correctly (shortest angle)
//         state_error(2, 0) = Normalize(state_error(2, 0));
//         Matrix state_cost_term = state_error.transpose() * Q * state_error;
//         total_cost += state_cost_term(0, 0);

//         // Input Magnitude Cost u^T * R * u
//         Matrix input_cost_term = inputs[k].transpose() * R * inputs[k];
//         total_cost += input_cost_term(0, 0);

//         // Input Rate-of-Change Cost (u_k - u_{k-1})^T * S * (u_k - u_{k-1})
//         if (k > 0) {
//           Matrix input_delta = inputs[k] - inputs[k-1];
//           Matrix input_rate_cost_term = input_delta.transpose() * S * input_delta;
//           total_cost += input_rate_cost_term(0, 0);
//         } else { // Cost relative to previous actual input
//           Matrix input_delta = inputs[k] - prev_input; // Need the input applied in the previous real step
//           Matrix input_rate_cost_term = input_delta.transpose() * S * input_delta;
//           // Optional: Could weight this differently or use the first element of the previously optimized sequence
//           total_cost += input_rate_cost_term(0, 0);
//         }
//       }
//     return total_cost;
//   }

//   /**
//   * Simulates the robot's path over the horizon given a sequence of inputs.
//   */
//   vector<Matrix> predict_trajectory(Matrix& start_state, vector<Matrix>& inputs) {
//     vector<Matrix> predicted_states;
//     predicted_states.reserve(PREDICTION_HORIZON);
//     Matrix current_pred_state = start_state;

//     for (int k = 0; k < PREDICTION_HORIZON; ++k) {
//       current_pred_state = predict_next_state(current_pred_state, inputs[k], TIMESTEP_DT);
//       predicted_states.push_back(current_pred_state);
//     }
//     return predicted_states;
//   }


//   /**
//   * Performs one iteration of gradient descent optimization.
//   * Uses numerical differentiation (finite differences).
//   */
//   void optimize_inputs(Matrix& current_state, vector<Matrix>& reference_sequence) {
//     vector<Matrix> current_inputs = current_input_sequence; // Work on a copy
//     for (int iter = 0; iter < OPTIMIZATION_ITERATIONS; ++iter) {
//       // Calculate gradient numerically
//       Matrix gradient(PREDICTION_HORIZON * INPUT_DIM, 1); // Flattened gradient vector
//       for (int k = 0; k < PREDICTION_HORIZON; ++k) { // For each timestep in horizon
//         for (int i = 0; i < INPUT_DIM; ++i) {       // For each input dimension (VL, VR)
//           // Calculate cost J(u)
//           vector<Matrix> predicted_states_base = predict_trajectory(current_state, current_inputs);
//           double cost_base = calculate_cost(predicted_states_base, current_inputs, reference_sequence);

//           // Perturb input u[k][i] slightly
//           vector<Matrix> perturbed_inputs = current_inputs;
//           perturbed_inputs[k](i, 0) += FINITE_DIFF_EPSILON;

//           // Calculate cost J(u + eps)
//           vector<Matrix> predicted_states_perturbed = predict_trajectory(current_state, perturbed_inputs);
//           double cost_perturbed = calculate_cost(predicted_states_perturbed, perturbed_inputs, reference_sequence);

//           // Numerical gradient: (J(u + eps) - J(u)) / eps
//           double grad_component = (cost_perturbed - cost_base) / FINITE_DIFF_EPSILON;
//           gradient(k * INPUT_DIM + i, 0) = grad_component;
//         }
//       }

//       // Apply gradient descent update: u = u - alpha * grad
//       for (int k = 0; k < PREDICTION_HORIZON; ++k) {
//         for (int i = 0; i < INPUT_DIM; ++i) {
//           current_inputs[k](i, 0) -= LEARNING_RATE * gradient(k * INPUT_DIM + i, 0);

//           // Clamp inputs to voltage limits
//           current_inputs[k](i, 0) = std::max(-MAX_VOLTAGE, std::min(MAX_VOLTAGE, current_inputs[k](i, 0)));
//         }
//       }
//     } // End optimization iterations

//     // Store the optimized sequence for the next iteration (warm start)
//     current_input_sequence = current_inputs;
//   }

//   /**
//   * Shifts the input sequence for the next timestep (removes first element, adds zero at end).
//   */
//   void shift_input_sequence() {
//     if (!current_input_sequence.empty()) {
//       current_input_sequence.erase(current_input_sequence.begin());
//       current_input_sequence.push_back(Matrix(INPUT_DIM, 1)); // Add zero input at the end
//     } else {
//       // Re-initialize if empty (shouldn't happen in normal operation)
//       current_input_sequence.resize(PREDICTION_HORIZON, Matrix(INPUT_DIM, 1));
//     }
//   }

//   Matrix computeControlAction(Matrix& current_state) {
//     // 1. Find where we are on the reference path
//     int closest_idx = findClosestTrajectoryPoint(current_state);

//     // 2. Get the reference states for the prediction horizon
//     vector<Matrix> reference_sequence = getReferenceSequence(closest_idx, current_state);

//     // 3. Optimize the input sequence using gradient descent
//     optimize_inputs(current_state, reference_sequence);

//     // 4. Get the first input from the optimized sequence
//     Matrix optimal_input_now = current_input_sequence[0];

//     // 5. Shift the sequence for the next time step (warm start)
//     shift_input_sequence();

//     return optimal_input_now;
//   }
































//   void setupTrajectoryAndMPC(void) {
//     // 1. Define Waypoints (X, Y in inches)
//     vector<Point> waypoints = {
//         Point(0, 0),
//         Point(24, 12),
//         Point(36, 36),
//         Point(12, 48),
//         Point(0, 24) // Loop back near start
//         // Add more points as needed
//     };

//     // Ensure at least 4 points for CatmullRom (including ghost points logic handles this)
//     if (waypoints.size() < 2) {
//          Brain.Screen.print("Error: Need at least 2 waypoints for trajectory.");
//          return; // Or handle error appropriately
//     }
//     // If exactly 2 or 3, add duplicates or reflect points to satisfy CatmullRom input needs
//     // The CatmullRom constructor provided now handles ghost points internally.


//     // 2. Create Catmull-Rom Spline
//     // Alpha = 0.5 for centripetal Catmull-Rom (usually good)
//     CatmullRom path(waypoints);

//     // 3. Define Robot Constraints
//     // Max Vel (in/s), Max Accel (in/s^2), Friction Coeff (tune!), Max Decel (in/s^2), Track Width (in)
//     // These constraints are used by the ProfileGenerator, not directly by MPC cost func here
//     // but they shape the reference trajectory velocity profile.
//     Constraints robotConstraints(
//         60.0,  // Max linear velocity (inches/sec) - TUNE
//         50.0,  // Max linear acceleration (inches/sec^2) - TUNE
//         0.7,   // Estimated coefficient of friction - TUNE
//         50.0,  // Max linear deceleration (inches/sec^2) - TUNE
//         TRACK_WIDTH_IN
//     );

//     // 4. Generate the Velocity Profile / Reference Trajectory
//     // ProfileGenerator(Constraints* constraints, double dd)
//     // dd = step distance (inches) for profile generation. Smaller = smoother but more points.
//     double profile_dd = 0.5; // Generate a point every 0.5 inches along the path
//     ProfileGenerator profileGen(&robotConstraints, profile_dd);
//     profileGen.generateProfile(path);
//     vector<ProfilePoint> generatedProfile = profileGen.getProfile();

//     if (generatedProfile.empty()) {
//          Brain.Screen.print("Error: Profile generation failed.");
//          return;
//     }

//     full_trajectory_profile = generatedProfile;
// }

// // // --- Main Control Task ---
// // int mpcControlTask() {
// //     Brain.Screen.print("MPC Control Task Started.");
// //     Brain.Screen.newLine();

// //     // Wait for MPC to be initialized
// //     while (!mpc) {
// //         task::sleep(20);
// //     }

// //     // Control Loop
// //     while (!trajectory_finished) {
// //         timer loopTimer; // Time the loop

// //         // 1. Get Current State
// //         Pose current_pose = getCurrentPose();
// //         double vL_rpm = getLeftWheelVelocityRPM();
// //         double vR_rpm = getRightWheelVelocityRPM();

// //         StateVector current_state(STATE_DIM, 1);
// //         current_state(0, 0) = current_pose.x;
// //         current_state(1, 0) = current_pose.y;
// //         current_state(2, 0) = current_pose.theta;
// //         current_state(3, 0) = vL_rpm;
// //         current_state(4, 0) = vR_rpm;

// //         // Check if we've reached the end (e.g., distance to last point is small)
// //         // Add proper end condition check here based on your needs.
// //         // Example: Check distance to the last point in `mpc->full_trajectory_profile`
// //         // if (distance_to_end < threshold) trajectory_finished = true;

// //         // 2. Compute Optimal Control Action
// //         Matrix optimal_action = mpc->computeControlAction(current_state);
// //         double optimal_voltL = optimal_action(0, 0);
// //         double optimal_voltR = optimal_action(1, 0);

// //         // 3. Apply Control Action to Motors
// //         setMotorVoltages(optimal_voltL, optimal_voltR);

// //         // 4. Debugging Output (Optional)
// //         Brain.Screen.setCursor(4, 1);
// //         Brain.Screen.print("State: X:%.1f Y:%.1f T:%.1f", current_pose.x, current_pose.y, current_pose.theta * 180/M_PI);
// //         Brain.Screen.newLine();
// //         Brain.Screen.print("Vels: L:%.1f R:%.1f RPM", vL_rpm, vR_rpm);
// //         Brain.Screen.newLine();
// //         Brain.Screen.print("Cmd V: L:%.1f R:%.1f V", optimal_voltL, optimal_voltR);
// //         Brain.Screen.newLine();
// //         Brain.Screen.print("Loop Time: %d ms", loopTimer.time(msec));
// //         Brain.Screen.clearLine(8); // Clear potential leftover chars

// //         // 5. Wait for next timestep
// //         int loop_time_ms = loopTimer.time(msec);
// //         int delay_ms = static_cast<int>(TIMESTEP_DT * 1000.0) - loop_time_ms;
// //         if (delay_ms > 0) {
// //              task::sleep(delay_ms);
// //         } else {
// //              Brain.Screen.print("Warning: MPC loop exceeded dt!");
// //              Brain.Screen.newLine();
// //              task::sleep(1); // Minimum delay
// //         }
// //     }

// //     // Trajectory finished, stop motors
// //     setMotorVoltages(0, 0);
// //     Brain.Screen.print("MPC Trajectory Finished.");
// //     Brain.Screen.newLine();
// //     return 0;
// // }




// }





















// // // -------- Constants --------
// // // --- Robot Physical Parameters (NEEDS TUNING/MEASUREMENT) ---
// // const double WHEEL_DIAMETER = 4.0; // inches
// // const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
// // const double TRACK_WIDTH = 12.0; // inches (distance between wheel centers)
// // const double GEAR_RATIO = 1.0; // Motor gear ratio (e.g., 1.0 for direct drive, 36.0/18.0 for 2:1)

// // // --- Unit Conversions ---
// // const double RPM_TO_IPS = (WHEEL_CIRCUMFERENCE / 60.0) / GEAR_RATIO; // RPM to Inches Per Second
// // const double IPS_TO_RPM = 1.0 / RPM_TO_IPS;
// // const double RPM_TO_RADPS = (2.0 * M_PI / 60.0) / GEAR_RATIO; // RPM to Radians Per Second (for angular velocity calculation)

// // // --- Control Limits (NEEDS TUNING/DATASHEET) ---
// // const double MAX_VOLTAGE = 12.0; // Volts
// // const double MAX_MOTOR_RPM = 200.0; // Example: V5 Smart Motor (Green Cartridge)
// // // Estimate max acceleration (RPM per second) - HIGHLY DEPENDENT on load, friction, etc. Needs experimental tuning.
// // // A very rough estimate might relate to how quickly voltage can change velocity.
// // const double MAX_RPM_ACCEL_PER_SEC = 500.0; // RPM/s - NEEDS HEAVY TUNING

// // // --- MPC Parameters ---
// // const int PREDICTION_HORIZON = 20; // N: Number of steps to predict ahead
// // const double TIMESTEP_DT = 0.02; // seconds (e.g., 20ms control loop)
// // // const double MAX_DELTA_VOLTAGE = (MAX_RPM_ACCEL_PER_SEC * IPS_TO_RPM / 5) * TIMESTEP_DT; // Limit voltage change per step - NEEDS TUNING/DERIVATION

// // // --- Optimization Parameters (NEEDS TUNING) ---
// // const double LEARNING_RATE = 0.1; // Step size for gradient descent
// // const int GRADIENT_ITERATIONS = 5; // Number of optimization iterations per timestep
// // const double GRADIENT_EPSILON = 0.01; // Small perturbation for numerical gradient calculation

// // // --- Cost Function Weights (NEEDS HEAVY TUNING) ---
// // // State Errors
// // const double Q_X = 5.0;       // Weight for X position error
// // const double Q_Y = 5.0;       // Weight for Y position error
// // const double Q_THETA = 1.0;   // Weight for Theta orientation error
// // const double Q_VL = 0.1;      // Weight for Left Velocity error
// // const double Q_VR = 0.1;      // Weight for Right Velocity error
// // // Input Magnitudes
// // const double R_VOLT_L = 0.01; // Weight penalizing Left Voltage magnitude
// // const double R_VOLT_R = 0.01; // Weight penalizing Right Voltage magnitude
// // // Input Rate of Change (Acceleration Proxy)
// // const double R_DELTA_VOLT_L = 0.05; // Weight penalizing change in Left Voltage
// // const double R_DELTA_VOLT_R = 0.05; // Weight penalizing change in Right Voltage


// // // -------- Structs --------
// // struct State {
// //   double x = 0.0;     // inches
// //   double y = 0.0;     // inches
// //   double theta = 0.0; // radians
// //   double vL = 0.0;    // RPM
// //   double vR = 0.0;    // RPM
// // };

// // struct Input {
// //   double voltL = 0.0; // Volts
// //   double voltR = 0.0; // Volts
// // };

// // // -------- Helper Functions --------

// // // Calculate the smallest difference between two angles
// // double angle_diff(double angle1, double angle2) {
// //   return Normalize(angle1 - angle2);
// // }

// // // Clamp a value between min and max
// // double clamp(double val, double min_val, double max_val) {
// //   return std::max(min_val, std::min(val, max_val));
// // }

// // // -------- Robot Dynamics Model --------
// // // Predicts the next state given the current state and input, over dt seconds
// // // THIS IS A SIMPLIFIED MODEL - NEEDS REFINEMENT/TUNING
// // State predictNextState(const State& current_state, const Input& input, double dt) {
// //   State next_state = current_state;

// //   // --- 1. Voltage to Velocity Change (Simplified Model) ---
// //   // This model assumes voltage roughly maps to a target velocity,
// //   // and the motor approaches it based on current velocity. Needs tuning.
// //   // A better model would involve motor constants (Kv, Ka), friction, inertia.

// //   // Simple Gain: Map max voltage to roughly max RPM (very approximate)
// //   const double VOLTS_TO_RPM_GAIN = MAX_MOTOR_RPM / MAX_VOLTAGE;

// //   // Target RPM based on voltage (ignoring friction/load for simplicity here)
// //   double target_rpm_L = input.voltL * VOLTS_TO_RPM_GAIN;
// //   double target_rpm_R = input.voltR * VOLTS_TO_RPM_GAIN;

// //   // Simulate acceleration limit (crude approximation)
// //   // How fast RPM can change towards the target RPM
// //   double max_rpm_change = MAX_RPM_ACCEL_PER_SEC * dt;

// //   double delta_rpm_L = clamp(target_rpm_L - current_state.vL, -max_rpm_change, max_rpm_change);
// //   double delta_rpm_R = clamp(target_rpm_R - current_state.vR, -max_rpm_change, max_rpm_change);

// //   next_state.vL = clamp(current_state.vL + delta_rpm_L, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
// //   next_state.vR = clamp(current_state.vR + delta_rpm_R, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);

// //   // --- 2. Kinematics: Velocity to Position/Orientation Change ---
// //   // Convert average RPM to linear/angular velocities
// //   double avg_vL_ips = (current_state.vL + next_state.vL) / 2.0 * RPM_TO_IPS;
// //   double avg_vR_ips = (current_state.vR + next_state.vR) / 2.0 * RPM_TO_IPS;

// //   double linear_vel_ips = (avg_vL_ips + avg_vR_ips) / 2.0;
// //   double angular_vel_radps = (avg_vR_ips - avg_vL_ips) / TRACK_WIDTH; // rad/s

// //   // --- 3. Integration: Update Pose ---
// //   // Use current theta for change calculation (Euler integration)
// //   double delta_x = linear_vel_ips * cos(current_state.theta) * dt;
// //   double delta_y = linear_vel_ips * sin(current_state.theta) * dt;
// //   double delta_theta = angular_vel_radps * dt;

// //   next_state.x = current_state.x + delta_x;
// //   next_state.y = current_state.y + delta_y;
// //   next_state.theta = Normalize(current_state.theta + delta_theta);

// //   return next_state;
// // }

// // // -------- MPC Core Logic --------

// // namespace MPC {

// //   vector<ProfilePoint> reference_profile; // Generated by ProfileGenerator
// //   State current_state;
// //   Input last_applied_input;

// //   // Helper to find the index of the closest point on the reference profile
// //   int findClosestPointIndex(const State& X) {
// //     double min_dist_sq = std::numeric_limits<double>::max();
// //     int closest_idx = 0;
// //     for (int i = 0; i < reference_profile.size(); ++i) {
// //       double dx = X.x - reference_profile[i].x;
// //       double dy = X.y - reference_profile[i].y;
// //       double dist_sq = dx * dx + dy * dy;
// //       if (dist_sq < min_dist_sq) {
// //         min_dist_sq = dist_sq;
// //         closest_idx = i;
// //       }
// //     }
// //     return closest_idx;
// //   }

// //   // Generate the reference states for the prediction horizon
// //   vector<State> getReferenceTrajectory(int closest_idx) {
// //     vector<State> ref_traj(PREDICTION_HORIZON);
// //     double current_dist_along_path = reference_profile[closest_idx].dist;

// //     for (int i = 0; i < PREDICTION_HORIZON; ++i) {
// //       // Estimate distance traveled in the next step based on profile velocity
// //       double estimated_vel = 0.0;
// //       if (closest_idx < reference_profile.size()) {
// //         estimated_vel = reference_profile[closest_idx].vel; // inches per second from profile
// //       } else if (!reference_profile.empty()) {
// //         estimated_vel = reference_profile.back().vel; // Use last velocity if near end
// //       }
// //       // Clamp estimated velocity to prevent overshoot if dt is large or vel high
// //       estimated_vel = std::max(0.0, estimated_vel);

// //       double distance_to_advance = estimated_vel * TIMESTEP_DT;
// //       current_dist_along_path += distance_to_advance;

// //       // Find the point on the profile corresponding to this new distance
// //       // (Simple linear interpolation or find next closest index)
// //       while (closest_idx + 1 < reference_profile.size() && reference_profile[closest_idx + 1].dist < current_dist_along_path) {
// //         closest_idx++;
// //       }

// //       // Now closest_idx is the last point *before* or *at* our target distance
// //       if (closest_idx < reference_profile.size()) {
// //         const auto& p = reference_profile[closest_idx];
// //         ref_traj[i].x = p.x;
// //         ref_traj[i].y = p.y;
// //         ref_traj[i].theta = p.theta; // Use profile's orientation

// //         // Convert profile linear/angular velocity back to wheel RPMs
// //         double linear_v = p.vel; // inches/sec
// //         double angular_w = p.omega; // radians/sec

// //         double ref_vR_ips = linear_v + (angular_w * TRACK_WIDTH / 2.0);
// //         double ref_vL_ips = linear_v - (angular_w * TRACK_WIDTH / 2.0);

// //         ref_traj[i].vL = ref_vL_ips * IPS_TO_RPM;
// //         ref_traj[i].vR = ref_vR_ips * IPS_TO_RPM;
// //       } else if (!reference_profile.empty()) {
// //         // If we projected past the end, use the last point
// //         ref_traj[i] = ref_traj[i > 0 ? i-1 : 0]; // Hold last reference state
// //         ref_traj[i].vL = 0; // Assume stop at end
// //         ref_traj[i].vR = 0;
// //       }
// //       // Else: If profile is empty, ref_traj remains default (0s)
// //     }
// //     return ref_traj;
// //   }


// //   // Calculate the total cost for a sequence of predicted states and inputs
// //   double calculateCost(const vector<State>& predicted_states, const vector<Input>& control_inputs, const vector<State>& reference_states) {
// //     double total_cost = 0.0;
// //     Input prev_input = last_applied_input; // Input from the step before horizon start
// //     for (int i = 0; i < PREDICTION_HORIZON; ++i) {
// //       const State& pred = predicted_states[i];
// //       const State& ref = reference_states[i];
// //       const Input& input = control_inputs[i];

// //       // --- State Tracking Cost ---
// //       double x_error = pred.x - ref.x;
// //       double y_error = pred.y - ref.y;
// //       double theta_error = angle_diff(pred.theta, ref.theta);
// //       double vL_error = pred.vL - ref.vL;
// //       double vR_error = pred.vR - ref.vR;

// //       total_cost += Q_X * x_error * x_error;
// //       total_cost += Q_Y * y_error * y_error;
// //       total_cost += Q_THETA * theta_error * theta_error;
// //       total_cost += Q_VL * vL_error * vL_error;
// //       total_cost += Q_VR * vR_error * vR_error;

// //       // --- Input Magnitude Cost ---
// //       total_cost += R_VOLT_L * input.voltL * input.voltL;
// //       total_cost += R_VOLT_R * input.voltR * input.voltR;

// //       // --- Input Rate of Change Cost ---
// //       double delta_voltL = input.voltL - prev_input.voltL;
// //       double delta_voltR = input.voltR - prev_input.voltR;
// //       total_cost += R_DELTA_VOLT_L * delta_voltL * delta_voltL;
// //       total_cost += R_DELTA_VOLT_R * delta_voltR * delta_voltR;

// //       // Update previous input for next iteration's delta calculation
// //       prev_input = input;
// //     }
// //     return total_cost;
// //   }


// //   // Optimize the control sequence using numerical gradient descent
// //   vector<Input> optimizeControls_GradientDescent(const State& initial_state, const vector<State>& reference_trajectory) {
// //     // Initialize control sequence (e.g., with zeros or repeat last input)
// //     vector<Input> current_inputs(PREDICTION_HORIZON, last_applied_input); // Start guess

// //     // Gradient vector (flattened: N * 2 elements)
// //     vector<double> gradient(PREDICTION_HORIZON * 2);

// //     for (int iter = 0; iter < GRADIENT_ITERATIONS; ++iter) {
// //       // Predict states based on current_inputs
// //       vector<State> predicted_states(PREDICTION_HORIZON);
// //       State sim_state = initial_state;
// //       for(int i = 0; i < PREDICTION_HORIZON; ++i) {
// //         predicted_states[i] = predictNextState(sim_state, current_inputs[i], TIMESTEP_DT);
// //         sim_state = predicted_states[i];
// //       }
// //       double base_cost = calculateCost(predicted_states, current_inputs, reference_trajectory);
// //       // Calculate numerical gradient
// //       for (int i = 0; i < PREDICTION_HORIZON; ++i) {
// //           // --- Gradient for Left Voltage at step i ---
// //           vector<Input> inputs_plus_L = current_inputs;
// //           inputs_plus_L[i].voltL += GRADIENT_EPSILON;
// //             // Clamp voltage before prediction
// //           inputs_plus_L[i].voltL = clamp(inputs_plus_L[i].voltL, -MAX_VOLTAGE, MAX_VOLTAGE);

// //           vector<Input> inputs_minus_L = current_inputs;
// //           inputs_minus_L[i].voltL -= GRADIENT_EPSILON;
// //           inputs_minus_L[i].voltL = clamp(inputs_minus_L[i].voltL, -MAX_VOLTAGE, MAX_VOLTAGE);


// //           vector<State> states_plus_L(PREDICTION_HORIZON);
// //           vector<State> states_minus_L(PREDICTION_HORIZON);
// //           State sim_state_plus_L = initial_state;
// //           State sim_state_minus_L = initial_state;

// //             for(int k=0; k < PREDICTION_HORIZON; ++k) {
// //               states_plus_L[k] = predictNextState(sim_state_plus_L, inputs_plus_L[k], TIMESTEP_DT);
// //               sim_state_plus_L = states_plus_L[k];

// //               states_minus_L[k] = predictNextState(sim_state_minus_L, inputs_minus_L[k], TIMESTEP_DT);
// //               sim_state_minus_L = states_minus_L[k];
// //           }
// //           double cost_plus_L = calculateCost(states_plus_L, inputs_plus_L, reference_trajectory);
// //           double cost_minus_L = calculateCost(states_minus_L, inputs_minus_L, reference_trajectory);
// //           gradient[i * 2 + 0] = (cost_plus_L - cost_minus_L) / (2.0 * GRADIENT_EPSILON);


// //           // --- Gradient for Right Voltage at step i ---
// //             vector<Input> inputs_plus_R = current_inputs;
// //           inputs_plus_R[i].voltR += GRADIENT_EPSILON;
// //           inputs_plus_R[i].voltR = clamp(inputs_plus_R[i].voltR, -MAX_VOLTAGE, MAX_VOLTAGE);


// //           vector<Input> inputs_minus_R = current_inputs;
// //           inputs_minus_R[i].voltR -= GRADIENT_EPSILON;
// //           inputs_minus_R[i].voltR = clamp(inputs_minus_R[i].voltR, -MAX_VOLTAGE, MAX_VOLTAGE);


// //           vector<State> states_plus_R(PREDICTION_HORIZON);
// //           vector<State> states_minus_R(PREDICTION_HORIZON);
// //             State sim_state_plus_R = initial_state;
// //           State sim_state_minus_R = initial_state;

// //           for(int k=0; k < PREDICTION_HORIZON; ++k) {
// //             states_plus_R[k] = predictNextState(sim_state_plus_R, inputs_plus_R[k], TIMESTEP_DT);
// //             sim_state_plus_R = states_plus_R[k];
// //             states_minus_R[k] = predictNextState(sim_state_minus_R, inputs_minus_R[k], TIMESTEP_DT);
// //             sim_state_minus_R = states_minus_R[k];
// //           }

// //           double cost_plus_R = calculateCost(states_plus_R, inputs_plus_R, reference_trajectory);
// //           double cost_minus_R = calculateCost(states_minus_R, inputs_minus_R, reference_trajectory);
// //           gradient[i * 2 + 1] = (cost_plus_R - cost_minus_R) / (2.0 * GRADIENT_EPSILON);
// //       } // End gradient calculation loop

// //     // Apply gradient descent update
// //     Input prev_input = last_applied_input;
// //     for (int i = 0; i < PREDICTION_HORIZON; ++i) {
// //       double new_voltL = current_inputs[i].voltL - LEARNING_RATE * gradient[i * 2 + 0];
// //       double new_voltR = current_inputs[i].voltR - LEARNING_RATE * gradient[i * 2 + 1];

// //       // --- Apply Constraints ---
// //       // 1. Max Voltage
// //         new_voltL = clamp(new_voltL, -MAX_VOLTAGE, MAX_VOLTAGE);
// //         new_voltR = clamp(new_voltR, -MAX_VOLTAGE, MAX_VOLTAGE);

// //         // 2. Max Delta Voltage (Rate of change / Acceleration proxy) - NEEDS TUNING
// //         // double delta_L = new_voltL - prev_input.voltL;
// //         // double delta_R = new_voltR - prev_input.voltR;
// //         // delta_L = clamp(delta_L, -MAX_DELTA_VOLTAGE, MAX_DELTA_VOLTAGE);
// //         // delta_R = clamp(delta_R, -MAX_DELTA_VOLTAGE, MAX_DELTA_VOLTAGE);
// //         // new_voltL = prev_input.voltL + delta_L;
// //         // new_voltR = prev_input.voltR + delta_R;

// //         // // Re-clamp after delta constraint
// //         // new_voltL = clamp(new_voltL, -MAX_VOLTAGE, MAX_VOLTAGE);
// //         // new_voltR = clamp(new_voltR, -MAX_VOLTAGE, MAX_VOLTAGE);


// //         current_inputs[i].voltL = new_voltL;
// //         current_inputs[i].voltR = new_voltR;
// //         prev_input = current_inputs[i]; // Update previous input for next step's delta constraint
// //       }
// //     } // End gradient descent iterations
// //     return current_inputs;
// //   }


// //   // Load the trajectory generated by ProfileGenerator
// //   void loadTrajectory(const vector<ProfilePoint>& profile) {
// //     reference_profile = profile;
// //     printf("Trajectory loaded with %d points.\n", reference_profile.size());
// //   }

// //   // Main computation step
// //   Input computeControlCommand(const State& current_robot_state) {
// //   if (reference_profile.empty()) {
// //     printf("Error: No reference profile loaded.\n");
// //     return Inputranspose(); // Return zero input if no trajectory
// //   }

// //   current_state = current_robot_state; // Update internal state

// //     // 1. Find the closest point on the reference path
// //     int closest_idx = findClosestPointIndex(current_state);

// //     // Optional: Check if we are "done" (close to the end)
// //     double dist_to_end_sq = pow(current_state.x - reference_profile.back().x, 2) + pow(current_state.y - reference_profile.back().y, 2);
// //     if (closest_idx >= reference_profile.size() - 2 && dist_to_end_sq < 1.0) { // Example threshold: 1 inch squared
// //       printf("Near end of trajectory.\n");
// //       // Maybe return zero voltage or hold last point's velocity?
// //       // For now, let optimizer handle it, potentially driving towards last point.
// //       //return {0.0, 0.0};
// //     }
// //     // 2. Generate the reference trajectory for the horizon
// //     vector<State> reference_trajectory = getReferenceTrajectory(closest_idx);

// //     // 3. Optimize the control inputs
// //     vector<Input> optimal_inputs = optimizeControls_GradientDescent(current_state, reference_trajectory);

// //     // 4. Apply the first control input from the optimal sequence
// //     Input command = optimal_inputs[0];

// //     // Update the last applied input for the next iteration's cost calculation (delta V)
// //     last_applied_input = command;
// //     return command;
// //   }

// //   const vector<ProfilePoint>& getLoadedProfile() {
// //     return reference_profile;
// //   }
// // }


// // // -------- Example Usage in main.cpp --------

// // // Assume these exist and are configured
// // brain Brain;
// // controller Controller1;
// // motor LeftMotor = motor(PORT1, ratio18_1, false); // Adjust port, ratio, reverse
// // motor RightMotor = motor(PORT2, ratio18_1, true); // Adjust port, ratio, reverse

// // // Assume an Odometry/MCL system exists and provides these
// // // Replace with your actual localization source
// // State getCurrentStateFromSensors() {
// //      // **** THIS IS A PLACEHOLDER - REPLACE WITH YOUR ACTUAL ODOMETRY/MCL ****
// //     static State estimated_state = {0, 0, 0, 0, 0};

// //     // Example: Update velocities from motors
// //     estimated_state.vL = LeftMotor.velocity(rpm);
// //     estimated_state.vR = RightMotor.velocity(rpm);

// //      // Example: Simulate pose update based on velocity (replace with actual MCL/Odom update)
// //      // This simple simulation here WILL drift without external correction (like MCL)
// //      double avg_vL_ips = estimated_state.vL * RPM_TO_IPS;
// //      double avg_vR_ips = estimated_state.vR * RPM_TO_IPS;
// //      double linear_vel_ips = (avg_vL_ips + avg_vR_ips) / 2.0;
// //      double angular_vel_radps = (avg_vR_ips - avg_vL_ips) / TRACK_WIDTH;
// //      double dt = Brain.Timer.time(seconds) - Brain.Timer.previousTime(seconds); // Or use fixed TIMESTEP_DT if loop is timed

// //      estimated_state.x += linear_vel_ips * cos(estimated_state.theta) * dt;
// //      estimated_state.y += linear_vel_ips * sin(estimated_state.theta) * dt;
// //      estimated_state.theta = normalize_angle(estimated_state.theta + angular_vel_radps * dt);

// //      Brain.Timer.clear(); // Reset timer for next dt calculation (if using Brain.Timer)

// //     return estimated_state;
// //     // **** END PLACEHOLDER ****
// // }

// // // Global MPC controller instance
// // MPCController mpc;

// // int main() {
// //     vexcodeInitranspose(); // Initialize VEX components

// //     // --- 1. Define Waypoints ---
// //     vector<Point> waypoints = {
// //         Point(0, 0),
// //         Point(24, 24),
// //         Point(0, 48),
// //         Point(48, 48)
// //         // Add at least 4 points for CatmullRom
// //     };

// //     // --- 2. Setup Trajectory Generation ---
// //     try {
// //         CatmullRom path(waypoints); // Use default alpha=0.5

// //         // Define robot constraints (NEEDS TUNING)
// //         Constraints constraints(
// //             MAX_MOTOR_RPM * RPM_TO_IPS,   // max_vel (inches/sec)
// //             MAX_RPM_ACCEL_PER_SEC * RPM_TO_IPS, // max_acc (inches/sec^2) - VERY rough estimate
// //             0.7,                          // friction_coef (dimensionless, estimate for wheels on tile)
// //             MAX_RPM_ACCEL_PER_SEC * RPM_TO_IPS * 0.8, // max_dec (in/s^2, slightly less than accel?) - NEEDS TUNING
// //             TRACK_WIDTH                   // track_width (inches)
// //         );

// //         // Generate the profile
// //         double path_segment_resolution = 0.5; // inches - density of points along path
// //         ProfileGenerator profileGen(&constraints, path_segment_resolution);
// //         profileGen.generateProfile(path);

// //         // Load the profile into the MPC controller
// //         mpc.loadTrajectory(profileGen.getProfile());

// //         if (mpc.getLoadedProfile().empty()) {
// //              Brain.Screen.print("Failed to generate or load profile!");
// //              return 1;
// //         }

// //         Brain.Screen.print("Profile Generated and Loaded. Ready.");
// //         wait(1, seconds);
// //         Brain.Screen.clearScreen();

// //     } catch (const std::invalid_argument& e) {
// //         Brain.Screen.print("Error creating path: %s", e.whatranspose());
// //         printf("Error creating path: %s\n", e.whatranspose());
// //         return 1; // Exit if path generation fails
// //     } catch (...) {
// //          Brain.Screen.print("Unknown error during trajectory generation!");
// //          printf("Unknown error during trajectory generation!\n");
// //          return 1;
// //     }


// //     // --- 3. Main Control Loop ---
// //     Brain.Timer.resetranspose(); // For dt calculations if needed by state estimator
// //     while (true) {
// //         // Get current state from sensors/localization
// //         State current_state = getCurrentStateFromSensors();

// //         // Calculate the optimal control command
// //         Input command = mpc.computeControlCommand(current_state);

// //         // Send command to motors
// //         LeftMotor.spin(forward, command.voltL, volt);
// //         RightMotor.spin(forward, command.voltR, volt);

// //         // Optional: Display debug info
// //         Brain.Screen.setCursor(1, 1);
// //         Brain.Screen.print("X: %5.1f Y: %5.1f T: %5.1f", current_state.x, current_state.y, current_state.theta * 180.0/M_PI);
// //          Brain.Screen.setCursor(2, 1);
// //          Brain.Screen.print("vL: %5.1f vR: %5.1f", current_state.vL, current_state.vR);
// //          Brain.Screen.setCursor(3, 1);
// //          Brain.Screen.print("CmdL: %4.1fV CmdR: %4.1fV", command.voltL, command.voltR);
// //          Brain.Screen.newLine();


// //         // Wait for the next control cycle
// //         wait(TIMESTEP_DT * 1000, msec); // Use the defined timestep
// //     }

// //     return 0;
// // }