#pragma once
#include <random>

using namespace vex;

namespace DrivePID {
  // Circumfrence of the Wheel
  constexpr double circumOfWheel = 3.25 * M_PI /* * (4 / 3) */; 
  // Tuning Values
  constexpr double kP = .72, kI = 0, kD = .24;

  // PID Vars
  extern double target;
  extern double Error;
  extern double Integral;
  extern double PrevError;
  extern double derivative;
  extern double StartEncoderLeft;
  extern double StartEncoderRight;
  // Start Bool
  extern bool startDrivePID;
  //The max/min voltages that the motor will run at
  extern double MaxPower, MinPower, DrivePower;
  // Functions
  extern void DrivePidOn(double num, double Maximum, double Minimun = 2.4);
  extern void DrivePid(void);
}

namespace TurnPID {
  // Timeout
  extern timer Timeout;
  // Settings
  extern double TkP, TkI, TkD;
  extern double Exponent;
  extern double NewTarget;
  extern bool Accurate;
  extern double StartAngle;
  // Autonnomous Settings
  extern int First;
  extern double Position;
  extern double Error;
  extern double integral;
  extern double derivative;
  extern double PrevError;
  extern double StartRotation;
  extern vex::brakeType brak;
  // Bool to run the PID
  extern bool startTurnPID;
  // The max voltage that the motor will run at
  extern double Target;
  extern double MaxPower, MinPower, TurnPower;
  // Functions
  extern void TurnPidOn(double Monke, double Realism, double Min = 4, bool TurnAccuracy = true, vex::brakeType braking = brake);
  extern void TurnPid(void);
}