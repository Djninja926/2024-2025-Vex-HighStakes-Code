#include "vex.h"
#include <cmath>
#include "api.h"

namespace LadyBrown {
  // Motion Profile Variables
  double target;
  double kP;
  
  bool startLadyBrownMP = false;

  // Function declarations
  void StartLadyBrown(double num);
  void LadyBrownOne(void);
  void LadyBrownTwo(void);
  void LadyBrownHold(void);
  void LadyBrownProfile(void);
}

void LadyBrown::StartLadyBrown(double num) {
  target = num; // Target position in degrees
  kP = fabs(num - Lift.position(deg)) * .005;
  startLadyBrownMP = true;
}

void LadyBrown::LadyBrownOne(void) {
  target = 96; // Target position in degrees
  kP = fabs(target - Lift.position(deg)) * .005;
  startLadyBrownMP = true;
}

void LadyBrown::LadyBrownTwo(void) {
  target = 132; // Target position in degrees
  kP = fabs(target - Lift.position(deg)) * .005;
  startLadyBrownMP = true;
}

void LadyBrown::LadyBrownHold(void) {
  target = 448; // Target position in degrees
  kP = fabs(target - Lift.position(deg)) * .005;
  startLadyBrownMP = true;
}


void LadyBrown::LadyBrownProfile(void) {
  if (startLadyBrownMP) {
    // Error
    double positionError = target - Lift.position(deg);

    // Direction
    double dir = positionError > 0 ? 1 : -1;

    // Calculate maximum allowable velocity to decelerate to stop
    double maxAllowedVel = sqrt(2 * maxAcceleration * fabs(positionError));
    double desiredVel = dir * fmin(maxVelocity, maxAllowedVel);

    // Velocity error and desired acceleration
    double velError = desiredVel - Lift.velocity(dps);
    double desiredAccel = velError * kP; // Adjust acceleration using P

    // Clamp acceleration to max allowed
    desiredAccel = fmax(fmin(desiredAccel, maxAcceleration), -maxAcceleration);

    // Feedforward (velocity) and feedback (acceleration) terms
    double LiftPower = (desiredAccel * kV) + (desiredVel * kF);

    // Clamp output to MaxPower
    LiftPower = fmax(fmin(LiftPower, MaxPower), -MaxPower);

    LiftPower = dir > 0 ? LiftPower : LiftPower + GravityOffset;
    // cout << dir << endl;

    // Apply voltage
    if (fabs(positionError) < 1) {
      Lift.stop(hold);
      startLadyBrownMP = false;
    } else {
      Lift.spin(fwd, LiftPower, volt);
    }
  }
}

// // Default
// #include "vex.h"
// #include "cmath"
// #include "api.h"

// namespace LadyBrownPID {
//   // Tuning Values
//   double kP = 1;
//   double kD;
//   double kI;
//   // PID Vars
//   double target;
//   double Error;
//   double integral;
//   double prevError;
//   double derivative;
//   double StartRotation; // The starting positions of the encoder
//   // Start Bool
//   bool startLadyBrownPID = false;
//   // Acceleration Settings
//   int First;
//   //The max/min voltages that the motor will run at
//   double MaxPower;
//   double MinPower;
//   double LiftPower;
//   // Functions
//   void LadyBrownOne(void);
//   void LadyBrownTwo(void);

//   // Main Loop
//   void LadyBrownPid(void);
// }


// void LadyBrownPID::LadyBrownOne(void) {
//   // Sets the Turning amount in degrees
//   target = 59;

//   // Sets the max power and min power
//   MaxPower = 7.2, MinPower = 2.7;

//   // Tuning Values
//   kP = .18;
//   kI = 0;
//   kD = .96;

//   // Reset the acceleration and start the PID
//   startLadyBrownPID = true;
// }

// void LadyBrownPID::LadyBrownTwo(void) {
//   // Sets the Turning amount in degrees
//   target = 92;

//   // Sets the max power and min power
//   MaxPower = 7.2, MinPower = 2.7;

//   // Tuning Values
//   kP = .18;
//   kI = 0;
//   kD = .96;

//   // Reset the acceleration and start the PID
//   startLadyBrownPID = true;
// }

// void LadyBrownPID::LadyBrownPid(void) {
//   if (startLadyBrownPID) {
//     // Error
//     Error = target - LB.position(deg);

//     // Derivative
//     derivative = Error - prevError;

//     // Power
//     LiftPower = (Error * kP) + (derivative * kD);

//     // Limits the power of the motors so it doesnt go past the max or min
//     if (LiftPower > 0) {
//       if (LiftPower < MinPower) LiftPower = MinPower;
//       if (LiftPower > MaxPower) LiftPower = MaxPower;
//     } else if (LiftPower < 0) {
//       if (LiftPower > (-MinPower)) LiftPower = (-MinPower);
//       if (LiftPower < (-MaxPower)) LiftPower = (-MaxPower); 
//     }

//     // Spins the Wheels
//     if (Error <= 2 && Error >= -2) {
//       Lift.stop(hold);
//       startLadyBrownPID = false;
//     } else {
//       Lift.spin(fwd, LiftPower, volt);
//     }

//     // Setting the Prev Values
//     prevError = Error;
//   }
// }