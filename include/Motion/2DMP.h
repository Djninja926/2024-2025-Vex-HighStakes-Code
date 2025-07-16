#pragma once

#include <vector>
#include "Misc/Point.h"

using namespace vex;
using namespace std;

namespace PurePursuit {
  extern bool StartPurePursuit, CalcPath, IsClearing;
  extern int ForwardSide, Stop, Loop;
  extern double StartTime, CurrentTime, PrevTime;
  /*--------------------*/
  /*  Pre Pure Pursuit  */
  /*--------------------*/
  // Variables for the Point Injection Algo - 1st Algo
    extern double DisBetweenPoint;
    extern double RelativeDis;
    extern int AmountOfPoints;
  // Variables for Setting Curvature and Velocities - 2nd Algo
  // Curvature
    extern double X1, X2, X3, Y1, Y2, Y3;
    extern double K1, K2, B, A, Radius, Curvature, MaxSpeed, NextMaxSpeed;
  // Velocities
    extern double NewVel, MaxAccelFraction, MinSpeed, NextMinSpeed;
  /*---------------------*/
  /*  Pure Pursuit Algo  */
  /*---------------------*/
  // Statically Defined Vars
    extern bool DefaultLookingDis, NextDefaultLookingDis;
    extern double LookAheadDis; // Should be 12 to 25
    extern double DefaultLookAheadDis, NextDefaultLookAheadDis;
    extern int CurrentPoint; // Index of the Point on the path that the robot has intersected
    extern timer ClosestPointTimer;
  /*----------------------*/
  /*     Closest Point    */
  /*----------------------*/
    extern double TempDis;
    extern double DistanceToPoint;
  /*----------------------*/
  /*   Look Ahead Point   */
  /*----------------------*/
    extern double a, b, c, Discriminant;
    extern double T_Intersection, T1, T2;
    extern double FractionalIndex, PrevFractionalIndex;
    extern double Degree, DisToClosestPointFraction, PDLθ, PWDθ, PWLθ;
    extern double CurvatureDifference, LookAheadCurvature, MaxCurve;
    extern bool LookAheadPointFound;
  /*----------------------*/
  /*   Curvature of Arc   */
  /*----------------------*/
    extern double ArcCurvature;
    extern double StandardA, StandardB, StandardC;
    extern double HorizontalDisToPoint, Side;
  /*----------------------*/
  /*   Wheel Velocities   */
  /*----------------------*/
    extern double TargetVelocity; // Target Velocity of the closest point on the path
    extern double LeftTargetVelocity; // Target Left side speed
    extern double RightTargetVelocity; // Target Right side speed
    extern double CurveOfArc, Fraction; // Curve of the arc the robot need to take to the lookahead point
    extern double TrackWidth; // 16 - 22 // the horizontal distance between the wheels on the robot. Due to turning scrub, you want to use a track width a few inches larger than the real one.
    // FeedForward Vars
    extern double LeftFeedForward, RightFeedForward;
    extern double LeftTargetAccel, RightTargetAccel, PrevLeftTargetVelocity, PrevRightTargetVelocity;
    // Feedback Vars
    extern double LeftMeasuredVelocity, RightMeasuredVelocity, PrevLeftMeasuredVelocity, PrevRightMeasuredVelocity;
    extern double LeftFeedBack, RightFeedBack;
    // Slew Rate Controller - Currently not using
    extern double LeftPower, RightPower;
    // Tuning Values
    // kV is the feedforward velocity constant, and it says how much power to apply for a given target velocity.
    // kA is the feedforward acceleration constant, does the same with acceleration.
    // kP is the proportional feedback constant.
    extern double kV, kA, kP;
  /*----------------------------*/
  /*   Globaly Defined Points   */
  /*----------------------------*/
  extern Point ClosestPoint, PrevClosestPoint; // Closest Point to the robot
  extern Point IntersectionPoint, LookAheadPoint, PrevLookAheadPoint;

  /* Vectors/Arrays of Points */
  extern vector<Point> PrimaryPoints;
  extern vector<Point> NewPoints;
  extern vector<Point> PathPoints;

  /* Vectors/Arrays of Misc Objects */
  extern vector<double> ClosestPointError;
  extern vector<double> Score;
  extern vector<double> DisToPath;

  /* Functions */
  // Void
  extern void ClearPath(void);
  extern void StartPath(void);
  extern void GetAccuracy(void);
  extern void SetIntakeFront(void);
  extern void SetClampFront(void);

  // Has Input
  extern void AddPath(std::vector<Point> Path);
  extern void CheckTimeOut(Point Close, Point Prev);
  extern void AddPoint(double PointX, double PointY);
  extern void TimedStop(int WaitTIme, brakeType Stopping = brake);
  extern void SetPath(double Spacing = 3, double MaximumSpeed = 12, double MinVelo = 0, double LookingAheadDis = 16, double K = 1, bool Smoothing = false, bool DefualtTracking = false);
  // extern void SetPath(double Spacing = 3, double MaximumSpeed = 12, double MinVelo = 0, double LookingAheadDis = 16, double K = 1, bool Smoothing = false, void (*Backup)() = ClearPath);

  // Double
  double GetLookAheadDis(double PrevLookAheadDis, double CurrentVeclocity, Point Close); // Based on Speed
  extern double GetLookAheadDis(double BaseLookAheadDis, Point Close, Point Next); // Based on Position

  // Point
  extern Point GetNewLookAheadPoint(Point FirstLookAheadPoint);

  // Main Loop
  extern void PurePursuitAlgo(void);
}



// Main Namespace
namespace Boom {
  extern double TargetX, TargetY, TargetAngle;
  extern double MaxVelo, MinVelo, Fraction;
  extern double CarrotPointX, CarrotPointY;
  extern double Dis, Ang, CurveRatio; // Can Mess With
  extern double AngularError, LinSpeed;
  extern double LinearKP, AngluarKP; // Still Needs to be Tuned // Tune by going straight making it as fast as possible then add the angular stuff
  extern double OverTurn, NewAngularError;
  extern double LeftPower, RightPower;
  extern double AngularSpeed, Tolerance, Duration, TimeoutDis;
  // Indicates End Angle or No End Angle
  extern bool EndPose;
  extern bool CanReverse;
  extern bool LeTimeoutState;
  // Timer
  extern timer Timeout;
  // Start Boomerang Variable
  extern bool StartBoomerang;
  // Functions
  extern void StopBoom(void);
  extern void MoveToPoint(int EndX, int EndY, double Max = 12, double Min = 2.4, bool Reverse = false);
  extern void MoveToPose(int EndX, int EndY, double EndAngle, double Max, double Min, bool Reverse);
  // Main Loop
  extern void Boomerang(void);
  extern double CalcTurnAngle(double TurnTargetX, double TurnTargetY);
}

// #pragma once

// #include "Misc/Matrix.h"
// #include "Misc/Point.h"
// #include "Misc/Spline.h"

// class Pose {
// public:
//     Pose(void) {
//       this->x = 0;
//       this->y = 0;
//       this->theta = 0;
//     }

//     Pose(double x, double y, double theta) {
//       this->x = x;
//       this->y = y;
//       this->theta = theta;
//     }
//     Pose(double x, double y) {
//       this->x = x;
//       this->y = y;
//       this->theta = 0;
//     }

//     double dist(Pose pose) {
//       return sqrt(pow(pose.x - this->x, 2) + pow(pose.y - this->y, 2));
//     }
//     double Angle(Pose pose) {
//       return atan2(pose.y - this->y, pose.x - this->x);
//     }

//     double x;
//     double y;
//     double theta;
// };

// class CatmullRom {
// public:
//   CatmullRom(vector<Point>& points) {
//     if (points.size() < 4) {
//       throw invalid_argument("CatmullRom requires at least 4 control points");
//     }

//     // Add ghost points at beginning and end
//     vector<Point> newPoints;
    
//     // First ghost point - reflect first point about second point
//     Point firstGhost = points[0] + (points[0] - points[1]);
//     newPoints.push_back(firstGhost);
    
//     // Add all original control points
//     for (const auto& point : points) {
//       newPoints.push_back(point);
//     }
    
//     // Last ghost point - reflect last point about second-to-last point
//     Point lastGhost = points.back() + (points.back() - points[points.size() - 2]);
//     newPoints.push_back(lastGhost);
    
//     // Store the augmented control points
//     this->controlPoints = newPoints;
//     /*  */
    
//     // this->controlPoints = points;
    
//     // Standard Catmull-Rom basis matrix
//     this->coefficients = Matrix(vector<vector<double>> { 
//       {-1,  3, -3, 1},
//       { 2, -5,  4, -1},
//       {-1,  0,  1, 0},
//       { 0,  2,  0, 0}
//     });
//     this->coefficients = this->coefficients * 0.5; // Multiply by 0.5 for standard Catmull-Rom
    
//     // Calculate segment knots and lengths
//     calculateSegmentInfo();
//   }

//   ~CatmullRom() {}

//   // Calculate which segment the given t value falls in
//   pair<int, double> getSegmentForT(double t) const {
//     if (t <= 0) return {0, 0.0};
//     if (t >= 1) return {numSegments() - 1, 1.0};
    
//     double scaledT = t * numSegments();
//     int segment = static_cast<int>(scaledT);
//     double localT = scaledT - segment;
    
//     return {segment, localT};
//   }

//   // Get actual number of curve segments
//   int numSegments() const {
//     return max(0, static_cast<int>(controlPoints.size()) - 3);
//   }

//   // Get four control points for a segment
//   vector<Point> getSegmentPoints(int segment) const {
//     vector<Point> segPoints;
//     for (int i = 0; i < 4; i++) {
//       segPoints.push_back(controlPoints[segment + i]);
//     }
//     return segPoints;
//   }

//   Point getPoint(double t) {
//     auto var = getSegmentForT(t);
//     int segment = var.first;
//     double localT = var.second;
//     return evaluateSegment(segment, localT);
//   }

//   Point getDerivative(double t) {
//     auto var = getSegmentForT(t);
//     int segment = var.first;
//     double localT = var.second;
//     return evaluateSegmentDerivative(segment, localT);
//   }

//   Point getSecondDerivative(double t) {
//     auto var = getSegmentForT(t);
//     int segment = var.first;
//     double localT = var.second;
//     return evaluateSegmentSecondDerivative(segment, localT);
//   }

//   double getCurvature(double t) {
//     Point d = this->getDerivative(t);
//     Point dd = this->getSecondDerivative(t);
//     return getCurvature(d, dd);
//   }

//   double getCurvature(Point d, Point dd) {
//     double k = (d.x * dd.y - d.y * dd.x) / pow(d.x * d.x + d.y * d.y, 1.5);
//     return k;
//   }

//   double getYaw(double t) {
//     if (t >= 0.999) {
//       Point dx = this->getPoint(t);
//       Point Start = this->getPoint(t - .01);
//       return atan2(dx.y - Start.y, dx.x - Start.x);
//     } else {
//       Point dx = this->getPoint(t + .01);
//       Point Start = this->getPoint(t);
//       return atan2(dx.y - Start.y, dx.x - Start.x);
//     }
//   }

//   // Get total number of control points
//   size_t getNumPoints() const {
//     return controlPoints.size();
//   }

// private:
//   vector<Point> controlPoints;
//   Matrix coefficients;
//   // Matrix derivativeCoef;
//   // Matrix secondDerivativeCoef;
//   // double alpha; // Parameterization control (0.5 for centripetal)
//   vector<double> segmentLengths;
  
//   void calculateSegmentInfo() {
//     segmentLengths.clear();
    
//     // Calculate approximate length of each segment for more uniform parameterization
//     for (int i = 0; i < numSegments(); i++) {
//       double segLength = 0.0;
//       Point prev = evaluateSegment(i, 0);
      
//       // Sample points along the segment to estimate length
//       const int samples = 10;
//       for (int j = 1; j <= samples; j++) {
//         double localT = static_cast<double>(j) / samples;
//         Point curr = evaluateSegment(i, localT);
//         segLength += sqrt(pow(curr.x - prev.x, 2) + pow(curr.y - prev.y, 2));
//         prev = curr;
//       }
      
//       segmentLengths.push_back(segLength);
//     }
//   }
  
//   Point evaluateSegment(int segment, double t) const {
//     if (segment < 0 || segment >= numSegments()) {
//       if (segment < 0) return controlPoints.front();
//       return controlPoints.back();
//     }
    
//     // Extract the four control points for this segment
//     vector<Point> segPoints = getSegmentPoints(segment);
    
//     // Create the matrix of control points
//     Matrix P(vector<vector<double>> {
//       {segPoints[0].x, segPoints[0].y},
//       {segPoints[1].x, segPoints[1].y},
//       {segPoints[2].x, segPoints[2].y},
//       {segPoints[3].x, segPoints[3].y}
//     });
    
//     // Calculate the point at t
//     Matrix T = Matrix({{t * t * t, t * t, t, 1}});
//     Matrix result = T * coefficients * P;
    
//     return Point(result(0, 0), result(0, 1));
//   }
  
//   Point evaluateSegmentDerivative(int segment, double t) const {
//     if (segment < 0 || segment >= numSegments()) {
//       return Point(0, 0); // Zero derivative at endpoints
//     }
    
//     // Extract the four control points for this segment
//     vector<Point> segPoints = getSegmentPoints(segment);
    
//     // Create the matrix of control points
//     Matrix P(vector<vector<double>> {
//       {segPoints[0].x, segPoints[0].y},
//       {segPoints[1].x, segPoints[1].y},
//       {segPoints[2].x, segPoints[2].y},
//       {segPoints[3].x, segPoints[3].y}
//     });
    
//     // Calculate the derivative at t
//     Matrix T = Matrix({{3 * t * t, 2 * t, 1, 0}});
//     Matrix result = T * coefficients * P;
    
//     return Point(result(0, 0), result(0, 1));
//   }
  
//   Point evaluateSegmentSecondDerivative(int segment, double t) const {
//     if (segment < 0 || segment >= numSegments()) {
//       return Point(0, 0); // Zero second derivative at endpoints
//     }
    
//     // Extract the four control points for this segment
//     vector<Point> segPoints = getSegmentPoints(segment);
    
//     // Create the matrix of control points
//     Matrix P(vector<vector<double>> {
//       {segPoints[0].x, segPoints[0].y},
//       {segPoints[1].x, segPoints[1].y},
//       {segPoints[2].x, segPoints[2].y},
//       {segPoints[3].x, segPoints[3].y}
//     });
    
//     // Calculate the second derivative at t
//     Matrix T = Matrix({{6 * t, 2, 0, 0}});
//     Matrix result = T * coefficients * P;
    
//     return Point(result(0, 0), result(0, 1));
//   }
// };

// class Constraints {
//   // Variables
//   public:
//   double max_vel;
//   double max_acc;
//   double friction_coef;
//   double max_dec;
//   double track_width;

//   public:
//     Constraints(void) {
//       this->max_vel = 0;
//       this->max_acc = 0;
//       this->friction_coef = 0;
//       this->max_dec = 0;
//       this->track_width = 0;
//     }

//     Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double track_width) {
//       this->max_vel = max_vel;
//       this->max_acc = max_acc;
//       this->friction_coef = friction_coef;
//       this->max_dec = max_dec;
//       this->track_width = track_width;
//     }

//     double maxSpeed(double curvature) {
//       double max_turn_speed = ((2 * this->max_vel / this->track_width) * this->max_vel) / (abs(curvature) * this->max_vel + (2 * this->max_vel / this->track_width));
//       if (curvature == 0) {
//         return max_turn_speed;
//       }

//       double max_slip_speed = sqrt(this->friction_coef * (1 / abs(curvature)) * 9.81 * 39.3701);
//       // double max_slip_speed = sqrt(this->friction_coef * (1 / abs(curvature)) * 3860.09);
//       // return max_turn_speed;
//       return min(max_slip_speed, max_turn_speed);
//     }

//     // pair<double, double> wheelSpeeds(double angularVel, double vel) {
//     //   double v_left = vel - angularVel * this->track_width / 2;
//     //   double v_right = vel + angularVel * this->track_width / 2;
//     //   return make_pair(v_left, v_right);
//     // }
//   };

// class ProfilePoint {
//   public:
//     ProfilePoint(double x, double y, double theta, double curvature, double t, double vel, double dist, double accel, double omega) {
//       this->x = x;
//       this->y = y;
//       this->theta = theta;
//       this->curvature = curvature;
//       this->t = t;
//       this->vel = vel;
//       this->dist = dist;
//       this->accel = accel;
//       this->omega = omega;
//     }

//     ProfilePoint(double dist, double vel, double omega) {
//       this->dist = dist;
//       this->vel = vel;
//       this->omega = omega;
//     }

//     double x;
//     double y;
//     double theta;
//     double curvature;
//     double t;
//     double vel;
//     double accel;
//     double dist;
//     double omega;
//   };

// class ChassisSpeeds {
// public:
//   ChassisSpeeds(double vel, double omega, double accel, Pose pose) {
//     this->vel = vel;
//     this->omega = omega;
//     this->accel = accel;
//     this->pose = pose;
//   }

//   double vel;
//   double omega;
//   double accel;
//   Pose pose;
// };

// class ProfileGenerator {
// public:
//   ProfileGenerator(Constraints *constraints, double dd) {
//     this->constraints = constraints;
//     this->dd = dd;
//   }
  
//   void generateProfile(CatmullRom path) {
//     this->profile.clear();
//     double dist = this->dd;

//     double vel = 0.00001;

//     vector<ProfilePoint> forwardPass;
//     vector<ProfilePoint> backwardPass;

//     forwardPass.push_back(ProfilePoint(0, 0, 0));
//     backwardPass.push_back(ProfilePoint(0, 0, 0));

//     double t = 0;
//     double curvature;
//     double angular_vel;
//     double angular_accel;
//     double last_angular_vel;
//     double max_accel;
//     double theta;
//     Point p;
//     Point deriv;
//     Point derivSecond;

//     while (t <= 1) {
//       p = path.getPoint(t);
//       // p.PrintPoint();
//       // vex::wait(1, vex::msec);
//       deriv = path.getDerivative(t);
//       derivSecond = path.getSecondDerivative(t);
//       t += dd / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);

//       curvature = .8 * path.getCurvature(deriv, derivSecond);
//       angular_vel = vel * curvature;
//       angular_accel = (angular_vel - last_angular_vel) * (vel / dd);
//       last_angular_vel = angular_vel;

//       max_accel = this->constraints->max_acc - abs(angular_accel * this->constraints->track_width / 2);
//       vel = min(this->constraints->maxSpeed(curvature), sqrt(vel * vel + 2 * max_accel * dd));
//       dist += dd;

//       theta = path.getYaw(t);

//       forwardPass.push_back(ProfilePoint(p.x, p.y, theta, curvature, t, vel, dist, 0, angular_vel));
//     }

//     vel = 0.00001;
//     last_angular_vel = 0;
//     angular_accel = 0;
//     t = 1;

//     while (t >= 0) {
//       p = path.getPoint(t);
//       // p.PrintPoint();
//       // vex::wait(1, vex::msec);
//       deriv = path.getDerivative(t);
//       derivSecond = path.getSecondDerivative(t);

//       t -= dd / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);
//       curvature = .8 * path.getCurvature(deriv, derivSecond);

//       angular_vel = vel * curvature;
//       angular_accel = (angular_vel - last_angular_vel) * (vel / dd);
//       last_angular_vel = angular_vel;

//       max_accel = this->constraints->max_dec - abs(angular_accel * this->constraints->track_width / 2);
//       vel = min(this->constraints->maxSpeed(curvature), sqrt(vel * vel + 2 * max_accel * dd));
//       dist -= dd;

//       theta = path.getYaw(t);

//       backwardPass.push_back(ProfilePoint(p.x, p.y, theta, curvature, t, vel, dist, 0, angular_vel));
//     }

//     // Get lower of the two velocities at each point and store in trajectory
//     for (int i = 0; i < backwardPass.size(); ++i) {
//       ProfilePoint lowerVel = forwardPass[i].vel < backwardPass[backwardPass.size() - i].vel ? forwardPass[i] : backwardPass[backwardPass.size() - i];
//       this->profile.push_back(ProfilePoint(forwardPass[i].x, forwardPass[i].y, forwardPass[i].theta, lowerVel.curvature, forwardPass[i].t, lowerVel.vel, forwardPass[i].dist, 0, lowerVel.omega));
//       // cout << forwardPass[i].x << endl;
//       // vex::wait(1, vex::msec);
//     }
//   }

//   ChassisSpeeds getProfilePoint(double d) {
//     int index = int(d / this->dd);
//     if (index >= this->profile.size()) {
//       index = this->profile.size() - 1;
//     }
//     double vel = this->profile[index].vel;
//     double angular_vel = this->profile[index].omega;
//     double accel = this->profile[index].accel;
//     return ChassisSpeeds(vel, angular_vel, accel, Pose(this->profile[index].x, this->profile[index].y, this->profile[index].theta));
//   }

//   vector<ProfilePoint> getProfile() { 
//     return profile;
//   }

// private:
//   Constraints *constraints;
//   vector<ProfilePoint> profile;
//   double dd;
// };

// namespace Ramsete {
//   // --- Parameters ---
//   extern const double dd;
//   extern double lmao;
//   // Tuning Gains (Adjust these based on testing)
//   extern const double b;     // Aggressiveness > 0 (higher values make convergence faster)
//   extern const double zeta;  // Damping ratio (0 < zeta < 1) (higher values reduce oscillation)
//   extern double trackWidth; // Inches - MAKE SURE THIS MATCHES CONSTRAINTS
//   extern const double MAX_SPEED; // Example: Max speed your robot can achieve in in/s
//   extern const double MAX_VOLTAGE;

//   // Control State
//   extern bool isFollowing;
//   extern int currentTrajectoryIndex;
//   extern vector<ProfilePoint> currentTrajectory;


//   /* Helper Function */
//   extern double sinc(double x);
//   extern double wheelSpeedToVoltage(double wheelSpeed_ips);

//  /* Main Function */

//   // Call this once to load the trajectory and start the control loop task
//   extern void startFollowing(const vector<ProfilePoint>& trajectoryToFollow);
//   extern void RamseteLoop(void);
// }