namespace LadyBrown {
  // Constants
  constexpr static double kV = .01;
  constexpr static double kF = .01;
  constexpr static double MaxPower = 12;
  constexpr static double GravityOffset = 2.0;
  constexpr static double maxVelocity = 1200.0;
  constexpr static double maxAcceleration = 2400.0;

  // Motion Profile Variables
  extern double target;
  extern double kP;
  
  extern bool startLadyBrownMP;

  // Function declarations
  extern void StartLadyBrown(double num);
  extern void LadyBrownOne(void);
  extern void LadyBrownTwo(void);
  extern void LadyBrownHold(void);
  extern void LadyBrownProfile(void);
}

// namespace LadyBrownPID {
//   // Tuning Values
//   extern double kP;
//   // PID Vars
//   extern double target;
//   extern double Error;
//   extern double integral;
//   extern double prevError;
//   extern double derivative;
//   extern double StartRotation; // The starting positions of the encoder
//   // Start Bool
//   extern bool startLadyBrownPID;
//   // Acceleration Settings
//   extern int First;
//   //The max/min voltages that the motor will run at
//   extern double MaxPower;
//   extern double MinPower;
//   extern double LiftPower;
//   // Functions
//   extern void LadyBrownOne(void);
//   extern void LadyBrownTwo(void);

//   // Main Loop
//   extern void LadyBrownPid(void);
// }