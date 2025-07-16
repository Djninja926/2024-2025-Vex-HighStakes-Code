// Default
#include "vex.h"
#include "cmath"
#include "api.h"

namespace DrivePID {
  // PID Vars
  double target;
  double Error;
  double Integral;
  double PrevError;
  double derivative;
  double StartEncoderLeft;
  double StartEncoderRight;
  // Start Bool
  bool startDrivePID;
  // Acceleration Settings
  //The max/min voltages that the motor will run at
  double MaxPower, MinPower, DrivePower;
  // Functions
  void DrivePidOn(double num, double Maximum, double Minimun);
  void DrivePid(void);
}

namespace TurnPID {
  // Timeout
  timer Timeout;
  // Settings
  double TkP, TkI, TkD;
  double Exponent = 0;
  double NewTarget = 0;
  bool Accurate = false;
  // Autonnomous Settings
  int First;
  double Position;
  double Error;
  double integral;
  double derivative;
  double PrevError;
  double StartRotation;
  vex::brakeType brak;
  // Bool to run the PID
  bool startTurnPID;
  // The max voltage that the motor will run at
  double Target;
  double MaxPower, MinPower, TurnPower;
  // Functions
  void TurnPidOn(double Monke, double Realism, double Min, bool TurnAccuracy, vex::brakeType braking);
  void TurnPid(void);
}



/*                     */
/*                     */
/*                     */


void DrivePID::DrivePidOn(double num, double Maximum, double Minimun) {
  // Setting the Target
  target = num;

  // Set the Power Limits
  MaxPower = Maximum;
  MinPower = Minimun;

  // Sets the Starting Encoder positions
  StartEncoderLeft = L1.position(degrees);
  StartEncoderRight = R1.position(degrees);

  // Reset the acceleration and start the PID
  startDrivePID = true;
}

void DrivePID::DrivePid(void) {
  if (startDrivePID) {
    // Average of the two encoders in degrees
    double posLeft = (L1.position(degrees) - StartEncoderLeft);
    double posRight = (R1.position(degrees) - StartEncoderRight);
    double avgPosi = (posLeft + posRight) / 2;

    // Converting both encoders from degrees to inches
    double encoderInch = ((circumOfWheel/360.0) * avgPosi * (3.0 / 4.0));
    // Potential
    Error = target - encoderInch;
    // Integral
    Integral += Error; if (Sign(Error) != Sign(PrevError)) Integral = 0;
    // Derivative
    derivative = Error - PrevError;
    // Drive Power of the PID in volts
    DrivePower = (Error * kP) + (Integral * kI) + (derivative * kD);

    PrevError = Error;
    //Limits the power of the motors so it doesnt go past the max
    if (target > 0) {
      if (DrivePower > MaxPower) DrivePower = MaxPower;  
      if (DrivePower < MinPower) DrivePower = MinPower;
    } else if (target < 0) {
      if (DrivePower < (-MaxPower)) DrivePower = (-MaxPower); 
      if (DrivePower > (-MinPower)) DrivePower = (-MinPower);
    }

    // Spins the Wheels
    // if (( (DrivePower <= MinPower && DrivePower >= -MinPower) && (Error <= 1 && Error >= -1)) || Timeout.time(msec) >= fabs(target) * Tolerance) {
    if (Error <= 1 && Error >= -1) {
      L1.stop(brake);
      L2.stop(brake);
      L3.stop(brake);
      R1.stop(brake);
      R2.stop(brake);
      R3.stop(brake);
      startDrivePID = false;
      this_thread::yield();
    } else {
      L1.spin(fwd, DrivePower, volt);
      L2.spin(fwd, DrivePower, volt);
      L3.spin(fwd, DrivePower, volt);
      R1.spin(fwd, DrivePower, volt);
      R2.spin(fwd, DrivePower, volt);
      R3.spin(fwd, DrivePower, volt);
    }
  }
}

void TurnPID::TurnPidOn(double Monke, double Realism, double Min, bool TurnAccuracy, vex::brakeType braking) {
  // Sets the Turning amount in degrees
  Target = Monke;
  // Sets the max power and min power
  brak = braking;
  MaxPower = Realism, MinPower = Min;
  // Setting Whether or not the Turn is Sped Up
  Accurate = TurnAccuracy;
  // Setting the Values
  if (!Accurate) {
    TkP = .84, TkI = 0, TkD = 0.0;
    NewTarget = fabs(Target);
    Exponent = (-0.000000000051608 * pow(NewTarget, 5)) + (0.00000002188 * pow(NewTarget, 4)) + (-0.00000313 * pow(NewTarget, 3)) + (0.000167 * pow(NewTarget, 2)) + (-0.00113 * NewTarget) + 0.498;
  } else {
    // OLD VALUES
    TkP = .16;
    TkI = 0;
    TkD = .96;
  }
  // Setting the Starting Odom Position
  StartRotation = ToDegrees(MCL::theta);
  // Reset the Timer
  Timeout.reset();
  // Reset the acceleration and start the PID
  startTurnPID = true;
}

// add something to normalize the error to try to make tuning easier, also set prev error
void TurnPID::TurnPid(void) {
  if (startTurnPID) {
    // The Rotations of the Inertial Sensor is Degrees
    // Potential
    if (Accurate) {
      Error = (Target - (ToDegrees(MCL::theta) - StartRotation));
    } else {
      Error = ((Target * Exponent) - (ToDegrees(MCL::theta) - StartRotation));
    }
    // Integral
    integral += Error; 
    if (Sign(Error) != Sign(PrevError)) integral = 0;
    // Derivative
    derivative = Error - PrevError;
    // Turn Power of the PID in volts
    TurnPower = ((Error * TkP)  + (integral * TkI) + (derivative * TkD));

    // Limits the power of the motors so it doesnt go past the max or min
    if (TurnPower > 0) {
      if (TurnPower < MinPower) TurnPower = MinPower;
      if (TurnPower > MaxPower) TurnPower = MaxPower;
    } else if (TurnPower < 0) {
      if (TurnPower > (-MinPower)) TurnPower = (-MinPower);
      if (TurnPower < (-MaxPower)) TurnPower = (-MaxPower); 
    }

    // Spins the Wheels
    if (((TurnPower <= MinPower && TurnPower >= -MinPower) && (Error <= 1 && Error >= -1) && Accurate) || ((TurnPower <= MinPower && TurnPower >= -MinPower) && (Error <= 1 && !Accurate)) || Timeout.time(msec) >= 1200) {
    // if (((TurnPower <= MinPower && TurnPower >= -MinPower) && (Error <= 1 && Error >= -1) && Accurate) || ((TurnPower <= MinPower && TurnPower >= -MinPower) && (Error <= 1 && !Accurate))) {
      L1.stop(brak); R1.stop(brak);
      L2.stop(brak); R2.stop(brak);
      L3.stop(brak); R3.stop(brak);
      startTurnPID = false;
      this_thread::yield();
    } else {
      L1.spin(fwd, TurnPower, volt); R1.spin(reverse, TurnPower, volt);
      L2.spin(fwd, TurnPower, volt); R2.spin(reverse, TurnPower, volt);
      L3.spin(fwd, TurnPower, volt); R3.spin(reverse, TurnPower, volt);
    }
    // Setting the Prev Values
    PrevError = Error;
  }
}