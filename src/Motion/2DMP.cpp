#include "cmath"
#include "vex.h"
#include "api.h"

using namespace std;

namespace PurePursuit {
  bool StartPurePursuit = false, CalcPath = false, IsClearing = false;
  int ForwardSide = 0, Stop = 0, Loop = 1;
  double StartTime, CurrentTime, PrevTime;
  /*--------------------*/
  /*  Pre Pure Pursuit  */
  /*--------------------*/
  // Variables for the Point Injection Algo - 1st Algo
    double DisBetweenPoint;
    double RelativeDis;
    int AmountOfPoints;
  // Variables for Setting Curvature and Velocities - 2nd Algo
  // Curvature
    double X1, X2, X3, Y1, Y2, Y3;
    double K1, K2, B, A, Radius, Curvature, MaxSpeed = 12.0, NextMaxSpeed;
  // Velocities
    double NewVel, MaxAccelFraction, MinSpeed = 1, NextMinSpeed;
  /*---------------------*/
  /*  Pure Pursuit Algo  */
  /*---------------------*/
  // Statically Defined Vars
    bool DefaultLookingDis, NextDefaultLookingDis;
    double LookAheadDis = 16; // Should be 12 to 25
    double DefaultLookAheadDis = 16, NextDefaultLookAheadDis;
    int CurrentPoint = 0; // Index of the Point on the path that the robot has intersected
    timer ClosestPointTimer;
  /*----------------------*/
  /*     Closest Point    */
  /*----------------------*/
    double TempDis = 225;
    double DistanceToPoint;
  /*----------------------*/
  /*   Look Ahead Point   */
  /*----------------------*/
    double a, b, c, Discriminant;
    double T_Intersection, T1, T2;
    double FractionalIndex, PrevFractionalIndex;
    double Degree, DisToClosestPointFraction, PDLθ, PWDθ, PWLθ;
    double CurvatureDifference = 0, LookAheadCurvature, MaxCurve = .2;
    bool LookAheadPointFound = false;
  /*----------------------*/
  /*   Curvature of Arc   */
  /*----------------------*/
    double ArcCurvature;
    double StandardA, StandardB, StandardC;
    double HorizontalDisToPoint, Side;
  /*----------------------*/
  /*   Wheel Velocities   */
  /*----------------------*/
    double TargetVelocity; // Target Velocity of the closest point on the path
    double LeftTargetVelocity; // Target Left side speed
    double RightTargetVelocity; // Target Right side speed
    double CurveOfArc, Fraction; // Curve of the arc the robot need to take to the lookahead point
    double TrackWidth = 13.5; // 16 - 22 // the horizontal distance between the wheels on the robot. Due to turning scrub, you want to use a track width a few inches larger than the real one.
    // FeedForward Vars
    double LeftFeedForward, RightFeedForward;
    double LeftTargetAccel, RightTargetAccel, PrevLeftTargetVelocity, PrevRightTargetVelocity;
    // Feedback Vars
    double LeftMeasuredVelocity, RightMeasuredVelocity;
    double LeftFeedBack, RightFeedBack;
    // Slew Rate Controller - Currently not using
    double LeftPower, RightPower;
    // Tuning Values
    // kV is the feedforward velocity constant, and it says how much power to apply for a given target velocity.
    // kA is the feedforward acceleration constant, does the same with acceleration.
    // kP is the proportional feedback constant.
    double kV = .8, kA = .2, kP = .6;
    // double kV = 1, kA = 0, kP = 0;
  /*----------------------------*/
  /*   Globaly Defined Points   */
  /*----------------------------*/
  Point ClosestPoint(0, 0), PrevClosestPoint(0, 0); // Closest Point to the robot
  Point IntersectionPoint(0, 0), LookAheadPoint(0, 0), PrevLookAheadPoint(0, 0);

  /* Vectors/Arrays of Points */
  vector<Point> PrimaryPoints;
  vector<Point> NewPoints;
  vector<Point> PathPoints;

  /* Vectors/Arrays of Misc Objects */
  vector<double> ClosestPointError;
  vector<double> Score;
  vector<double> DisToPath;

  /* Functions */
  // Void
  void ClearPath(void);
  void StartPath(void);
  void GetAccuracy(void);
  void SetIntakeFront(void);
  void SetClampFront(void);

  // Has Input
  void AddPath(vector<Point> Path);
  void TimedStop(Point TimeoutPoint);
  void CheckTimeOut(Point Close, Point Prev);
  void AddPoint(double PointX, double PointY);
  void TimedStop(int WaitTIme, brakeType Stopping);
  void SetPath(double Spacing, double MaximumSpeed, double MinVelo, double LookingAheadDis, double K, bool Smoothing, bool DefualtTracking);
  // void SetPath(double Spacing, double MaximumSpeed, double MinVelo, double LookingAheadDis, double K, bool Smoothing, void (*Backup)());

  // Double 
  double GetLookAheadDis(double PrevLookAheadDis, double CurrentVeclocity, Point Close); // Based on Speed
  double GetLookAheadDis(double BaseLookAheadDis, Point Close, Point Next); // Based on Position

  // Point
  Point GetNewLookAheadPoint(Point FirstLookAheadPoint);

  // Main Loops
  void PurePursuitAlgo(void);
}

/* ------------------ */
/* Pure Pursuit Algo  */
/* ------------------ */

void PurePursuit::SetPath(double Spacing, double MaximumSpeed, double MinVelo, double LookingAheadDis, double K, bool Smoothing, bool DefualtTracking) {
  CalcPath = true;
  Loop = 1;
  // MinSpeed = MinVelo;
  NextMinSpeed = MinVelo;
  // DefaultLookAheadDis = LookingAheadDis;
  NextDefaultLookAheadDis = LookingAheadDis;
  // Whether or not to use the new and improved tracking
  NextDefaultLookingDis = DefualtTracking;

  // Injecting All new points
  DisBetweenPoint = Spacing;
  for (int X = 0; X < PrimaryPoints.size() - 1; X++) {
    Point vec(PrimaryPoints.at(X + 1).GetX() - PrimaryPoints.at(X).GetX(), PrimaryPoints.at(X + 1).GetY() - PrimaryPoints.at(X).GetY());
    RelativeDis = sqrt(pow((PrimaryPoints.at(X + 1).GetX() - PrimaryPoints.at(X).GetX()), 2) + pow((PrimaryPoints.at(X + 1).GetY() - PrimaryPoints.at(X).GetY()), 2));
    AmountOfPoints = RelativeDis / Spacing;
    vec = Point(vec.GetX() / AmountOfPoints, vec.GetY() / AmountOfPoints);
    for (int Injection = 0; Injection < AmountOfPoints; Injection++) {
      Point NewPoint((PrimaryPoints.at(X).GetX() + (vec.GetX() * Injection)), (PrimaryPoints.at(X).GetY() + (vec.GetY() * Injection)));
      NewPoints.push_back(NewPoint);
    }
    wait(2, msec);
  }

  if (Smoothing) {
    double Tolerance = .001, b1 = .8;
    double a1 = 1 - b1;
    double Change = Tolerance;
    vector<Point> TempPoints = NewPoints;
    while (Change >= Tolerance) {
      Change = 0.0;
      for (int P = 1; P < NewPoints.size() - 1; P++) {
        double Xthing = NewPoints.at(P).GetX();
        double Ything = NewPoints.at(P).GetY();
        NewPoints.at(P) = Point(NewPoints.at(P).GetX() + (a1 * (TempPoints.at(P).GetX() - NewPoints.at(P).GetX()) + b1 * (NewPoints.at(P-1).GetX() + NewPoints.at(P+1).GetX() - (2.0 * NewPoints.at(P).GetX()))), NewPoints.at(P).GetY() + (a1 * (TempPoints.at(P).GetY() - NewPoints.at(P).GetY()) + b1 * (NewPoints.at(P-1).GetY() + NewPoints.at(P+1).GetY() - (2.0 * NewPoints.at(P).GetY()))));
        Change += fabs((Xthing - NewPoints.at(P).GetX()) + (Ything - NewPoints.at(P).GetY()));
      }
     wait(2, msec);
    }
  }
  
  NewPoints.push_back(PrimaryPoints.at(PrimaryPoints.size() - 1));
  wait(5, msec);
  
  // Setting All the Curvatures
  NextMaxSpeed = MaximumSpeed;
  for (int Y = 1; Y < NewPoints.size() - 1; Y++) {
  /* Finding the Curvature at each point */
    //Setting the X values of the Points
    X1 = NewPoints.at(Y).GetX() + .0001;
    X2 = NewPoints.at(Y - 1).GetX();
    X3 = NewPoints.at(Y + 1).GetX();
    // Setting the Y-values of the points
    Y1 = NewPoints.at(Y).GetY();
    Y2 = NewPoints.at(Y - 1).GetY();
    Y3 = NewPoints.at(Y + 1).GetY();
    // Setting the other stuff for curvature
    K1 = .5 * (pow(X1, 2) + pow(Y1, 2) - pow(X2, 2) - pow(Y2, 2)) / (X1 - X2);
    K2 = (Y1 - Y2)/(X1 - X2);
    B = .5 * (pow(X2, 2) - (2 * X2 * K1) + pow(Y2, 2) - pow(X3, 2) + (2 * X3 * K1) - pow(Y3, 2)) / ((X3 * K2) - Y3 + Y2 - (X2 * K2));
    A = K1 - (K2 * B);
    Radius = sqrt(pow((X1 - A), 2) + pow((Y1 - B), 2));
    Curvature = 1 / Radius;
    NewPoints.at(Y).SetVelocity(min(NextMaxSpeed, (K/Curvature))); // Test 1.44 without smoothing
    NewPoints.at(Y).SetCurve(Curvature);
    wait(2, msec);
  }


  // Setting all the Velocities
  NewPoints.at(0).SetVelocity(MaximumSpeed);  
  for (int Z = NewPoints.size() - 2; Z >= 0; Z--) {
    MaxAccelFraction = 1.25;
    NewVel = std::min(NewPoints.at(Z).GetVelocity(), std::sqrt(pow(NewPoints.at(Z + 1).GetVelocity(), 2) + (2 * MaxAccelFraction * DisBetweenPoint)));
    // NewVel = min(NewPoints.at(Z).GetVelocity(), sqrt(pow(NewPoints.at(Z + 1).GetVelocity(), 2) + (2 * MaxAccelFraction * sqrt(pow((NewPoints.at(Z + 1).GetX() - NewPoints.at(Z).GetX()), 2) + pow((NewPoints.at(Z + 1).GetY() - NewPoints.at(Z).GetY()), 2)))));
    NewPoints.at(Z).SetVelocity(NewVel);
    wait(2, msec);
  }
  NewPoints.at(NewPoints.size() - 1).SetVelocity(0);
  // NewPoints.push_back(NewPoints[NewPoints.size() - 1]);
  // NewPoints.push_back(Point(0, 0, 0, 0));
  // NewPoints.at(NewPoints.size() - 2).SetVelocity(0);
  PrimaryPoints.clear();
  CalcPath = false;
}

void PurePursuit::PurePursuitAlgo(void) {
  while (true) {
    if (StartPurePursuit) {
    Stop = 1, CurrentTime = Brain.timer(msec);
  /*------------------------------------------------------*/
  /*                    Closest Point                     */
  /*------------------------------------------------------*/

  TempDis = 225;
  
  for (int Monke = CurrentPoint; Monke < PathPoints.size(); Monke++) {
    DistanceToPoint = sqrt(pow((PathPoints.at(Monke).GetX() - MCL::X), 2) + pow((PathPoints.at(Monke).GetY() - MCL::Y), 2));
    if (DistanceToPoint < TempDis) {
      ClosestPoint = PathPoints.at(Monke);
      TempDis = DistanceToPoint;
      CurrentPoint = Monke;
    }
    // wait(1, msec);
  }
  // ClosestPointError.push_back(TempDis);
  /* Checking if it needs to timeout */
  CheckTimeOut(ClosestPoint, PrevClosestPoint);

  /*------------------------------------------------------*/
  /*                   Look Ahead Point                   */
  /*------------------------------------------------------*/
  // Setting Lookahead Dis
  if (!DefaultLookingDis) {
    // if (CurrentPoint > 0) {
    //   LookAheadDis = GetLookAheadDis(DefaultLookAheadDis, PathPoints.at(CurrentPoint + 1), ClosestPoint);
    // }
    if (CurrentPoint < PathPoints.size() - 1) {
      // LookAheadDis = GetLookAheadDis(DefaultLookAheadDis, ClosestPoint, PathPoints.at(CurrentPoint + 1));
      LookAheadDis = GetLookAheadDis(DefaultLookAheadDis, PathPoints.at(CurrentPoint + 1), ClosestPoint);
    }
  } else {
    LookAheadDis = DefaultLookAheadDis;
  }

  // LookAheadDis = DefaultLookAheadDis ? (CurrentPoint < PathPoints.size() - 1 ? GetLookAheadDis(DefaultLookAheadDis, ClosestPoint, PathPoints.at(CurrentPoint + 1)) : DefaultLookAheadDis) : DefaultLookAheadDis;
  // Reseting Lookahead Point Found
  LookAheadPointFound = false;
  for (int Seeker = CurrentPoint; Seeker < PathPoints.size() - 1; Seeker++) {
  // The first point of the line segment
  Point FirstPoint = PathPoints.at(Seeker);
  // The Ending point of the line segment
  Point NextPoint = PathPoints.at(Seeker + 1);
  // The Point the Robot is Currently At
  Point TrackingCenter(MCL::X, MCL::Y);
  // Sets T-intersection to zero for new calculations
  T_Intersection = 0;
  // Direction vector of ray, from start to end
  Point DirectionVector(NextPoint.GetX() - FirstPoint.GetX(), NextPoint.GetY() - FirstPoint.GetY());
  // Vector from center sphere to ray start
  Point CenterVector(FirstPoint.GetX() - TrackingCenter.GetX(), FirstPoint.GetY() - TrackingCenter.GetY());

  a = (DirectionVector.GetX() * DirectionVector.GetX()) + (DirectionVector.GetY() * DirectionVector.GetY());
  b = 2 * ((CenterVector.GetX() * DirectionVector.GetX()) + (CenterVector.GetY() * DirectionVector.GetY()));
  c = ((CenterVector.GetX() * CenterVector.GetX()) + (CenterVector.GetY() * CenterVector.GetY())) - pow(LookAheadDis, 2);
  Discriminant = pow(b, 2) - (4 * a * c);
  
  if (Discriminant < 0) {
    T_Intersection = 0;
  } else {
      Discriminant = sqrt(Discriminant);

      T1 = (-b - Discriminant) / (2 * a);
      T2 = (-b + Discriminant) / (2 * a);

      // Calculating the intersection
      if (T1 >= 0 && T1 <= 1) {
        // 1st intersection
        T_Intersection = T1;
      } else if (T2 >= 0 && T2 <= 1) {
        // 2nd intersection
        T_Intersection = T2;
      } else {
        // Other wise no intersection
        T_Intersection = 0;
      }
    }

    FractionalIndex = T_Intersection + Seeker;

    if (FractionalIndex > PrevFractionalIndex) {
      IntersectionPoint = Point((FirstPoint.GetX() + (T_Intersection * DirectionVector.GetX())), (FirstPoint.GetY() + (T_Intersection * DirectionVector.GetY())));
      PrevFractionalIndex = FractionalIndex;
      LookAheadPointFound = true;
      break;
    }

    wait(1, msec);
  }

  if (!LookAheadPointFound) {
    LookAheadPoint = PrevLookAheadPoint;
  } else {
    if (!DefaultLookingDis && CurrentPoint < PathPoints.size() - 1) {
      LookAheadPoint = GetNewLookAheadPoint(IntersectionPoint);
    } else {
      LookAheadPoint = IntersectionPoint;
    }
  }
  /*------------------------------------------------------*/
  /*                   Curvature of Arc                   */
  /*------------------------------------------------------*/
  StandardA = -tan(M_PI_2 - MCL::theta);
  StandardB = 1;
  StandardC = (tan(M_PI_2 - MCL::theta) * MCL::X) - MCL::Y;

  HorizontalDisToPoint = fabs((StandardA * LookAheadPoint.GetX()) + (StandardB * LookAheadPoint.GetY()) + StandardC) / sqrt(pow(StandardA, 2) + pow(StandardB, 2));
  
  ArcCurvature = (2 * (HorizontalDisToPoint)) / pow(LookAheadDis, 2);
  // Calculating if its on the Right Side or Left Side
  Side = Sign(sin(M_PI_2 - MCL::theta) * (LookAheadPoint.GetX() - MCL::X) - cos(M_PI_2 - MCL::theta) * (LookAheadPoint.GetY() - MCL::Y));

  // The Horizontal Dis of the closest point for scoring the path
  // HorizontalDisToPoint = fabs((StandardA * ClosestPoint.GetX()) + (StandardB * ClosestPoint.GetY()) + StandardC) / sqrt(pow(StandardA, 2) + pow(StandardB, 2));
  // Score.push_back(HorizontalDisToPoint);
  
  /*------------------------------------------------------*/
  /*                   Wheel Velocities                   */
  /*------------------------------------------------------*/
  
  TargetVelocity = ClosestPoint.GetVelocity(); // Target Velocity of the closest point on the path

  CurveOfArc = ArcCurvature * Side; // Curve of the arc the robot need to take to the lookahead point
  // Calculation of the Target velocites for each side

  LeftTargetVelocity = TargetVelocity * (2 + (CurveOfArc * TrackWidth)) / 2; // target left side speed
  RightTargetVelocity = TargetVelocity * (2 - (CurveOfArc * TrackWidth)) / 2; // target right side speed

  // Left Calculations
  LeftTargetAccel = (LeftTargetVelocity - PrevLeftTargetVelocity) / (CurrentTime - PrevTime);
  // The Feed forward (kV * LeftTargetVelocity) + a little extra boost (kA * LeftTargetAccel)
  LeftFeedForward = kV * LeftTargetVelocity + kA * LeftTargetAccel;
  // Setting the prev velocity for the next rendition of the loop
  PrevLeftTargetVelocity = LeftTargetVelocity;

  // Right Calculations
  RightTargetAccel = (RightTargetVelocity - PrevRightTargetVelocity) / (CurrentTime - PrevTime);
  // The Feed forward (kV * LeftTargetVelocity) + a little extra boost (kA * LeftTargetAccel)
  RightFeedForward = kV * RightTargetVelocity + kA * RightTargetAccel;
  // Setting the prev velocity for the next rendition of the loop
  PrevRightTargetVelocity = RightTargetVelocity;
  
  // FeedBack Calculations
  LeftMeasuredVelocity = (12.0/600.0) * (fabs(L1.velocity(rpm) + L2.velocity(rpm) + L3.velocity(rpm)) / 3);
  RightMeasuredVelocity = (12.0/600.0) * (fabs(R1.velocity(rpm) + R2.velocity(rpm) + R3.velocity(rpm)) / 3);

  // cout << "Left: " << LeftMeasuredVelocity << "\t\tRight: " << RightMeasuredVelocity << endl;

  // cout << TargetVelocity << endl;

  LeftFeedBack = (fabs(LeftTargetVelocity) - LeftMeasuredVelocity) * kP;
  RightFeedBack = (fabs(RightTargetVelocity) - RightMeasuredVelocity) * kP;

  LeftPower = LeftFeedForward + LeftFeedBack;
  RightPower = RightFeedForward + RightFeedBack;

  // cout << "Left: " << LeftPower << "\t\tRight: " << RightPower << endl;

  // Better way of limiting power
  Fraction = max(fabs(LeftPower), fabs(RightPower)) / MaxSpeed;
  // Fraction'ing the Powers
  if (Fraction > 1) LeftPower /= Fraction, RightPower /= Fraction;
  
  Fraction = min(fabs(LeftPower), fabs(RightPower)) / MinSpeed;
  // // Fraction'ing the Powers
  if (Fraction < 1) LeftPower /= Fraction, RightPower /= Fraction;

  // cout << "Left: " << LeftPower << "\t\tRight: " << RightPower << endl;

  if (TargetVelocity == 0) {
    L1.stop(brake);
    L2.stop(brake);
    L3.stop(brake);
    R1.stop(brake);
    R2.stop(brake);
    R3.stop(brake);
    ClearPath();
    StartPurePursuit = false;
    this_thread::yield();
  } else {
    if (ForwardSide == 0) {
      // Left Side
      L1.spin(fwd, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      L2.spin(fwd, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      L3.spin(fwd, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      // Right Side
      R1.spin(fwd, RightPower, volt); // RightFeedForward + RightFeedBack
      R2.spin(fwd, RightPower, volt); // RightFeedForward + RightFeedBack
      R3.spin(fwd, RightPower, volt); // RightFeedForward + RightFeedBack
    } else if (ForwardSide == 1) {
      // Left Side
      L1.spin(reverse, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      L2.spin(reverse, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      L3.spin(reverse, LeftPower, volt); // LeftFeedForward + LeftFeedBack
      // Right Side
      R1.spin(reverse, RightPower, volt); // RightFeedForward + RightFeedBack
      R2.spin(reverse, RightPower, volt); // RightFeedForward + RightFeedBack
      R3.spin(reverse, RightPower, volt); // RightFeedForward + RightFeedBack
    }
  }

    // Needs to be at the end cause new point everytime the loop runs
      PrevLookAheadPoint = LookAheadPoint, PrevClosestPoint = ClosestPoint; // Setting the Prev Points
      PrevTime = CurrentTime; // Setting Prev Time
      Stop = 0, Loop++;
      // Wait time for the loop
      wait(5, msec);
    }
    wait(5, msec);
  }
}

void PurePursuit::StartPath(void) {
  // Reseting the Timer
  ClosestPointTimer.reset();

  // Setting the Path
  PathPoints = NewPoints;

  // Setting all the Variables
  PrevFractionalIndex = 0, FractionalIndex = 0, CurrentPoint = 0, Loop = 1;
  MaxSpeed = NextMaxSpeed; MinSpeed = NextMinSpeed;
  DefaultLookAheadDis = NextDefaultLookAheadDis;
  DefaultLookingDis = NextDefaultLookingDis;

  // Clearing all the vectors
  ClosestPointError.clear();
  DisToPath.clear();
  Score.clear();
  NewPoints.clear();
  
  // Thread Printing all the Points
  // thread ([]{for (int x = 0; x < PathPoints.size(); x++) {PathPoints.at(x).PrintPoint(); wait(1, msec);} cout << "\n\n\n\n\n" << endl;}).detach();

  // Starting the Path
  StartPurePursuit = true;
}

void PurePursuit::AddPoint(double PointX, double PointY) {
  PrimaryPoints.push_back(Point(PointX, PointY));
}
void PurePursuit::AddPath(vector<Point> Path) {
  PrimaryPoints = Path;
}
void PurePursuit::SetIntakeFront(void) {
  ForwardSide = 0;
}
void PurePursuit::SetClampFront(void) {
  ForwardSide = 1;
}

void PurePursuit::TimedStop(int WaitTIme, brakeType Stopping) {
  StartTime = Brain.timer(msec);
  waitUntil(Brain.timer(msec) - StartTime > WaitTIme || !StartPurePursuit);
  if (StartPurePursuit) {
    StartPurePursuit = false;
    waitUntil(!Stop);
    L1.stop(Stopping), L2.stop(Stopping), L3.stop(Stopping); R1.stop(Stopping), R2.stop(Stopping), R3.stop(Stopping);
    ClearPath();
  }
}

void PurePursuit::TimedStop(Point TimeoutPoint) { // Make This
  
}

void PurePursuit::CheckTimeOut(Point Close, Point Prev) {
  if (Close != Prev) {
    ClosestPointTimer.reset();
  } else {
    if (ClosestPointTimer.time(msec) > 750) {
      ClosestPoint.SetVelocity(0);
      // __throw_runtime_error("It Broke");
    }
  }
}

Point PurePursuit::GetNewLookAheadPoint(Point FirstLookAheadPoint) {
  /* Calculating the Angles of the vectors */
  // Angle of the Vector of the Closest Point (P_W) to the First LookAhead Point (P_D)
  PWDθ = atan2(-1 * (ClosestPoint.GetX() - FirstLookAheadPoint.GetX()), ClosestPoint.GetY() - FirstLookAheadPoint.GetY()) * -1;
  // Wrap the target heading around 360 (0 - 360)
  if (PWDθ < 0) PWDθ += (2 * M_PI);

  // Angle of the Vector of the Closest Point (P_W) to the New LookAhead Point (P_L) - Since we don't have the new lookahead point, the angle is based of the fact that the vector is tangent to the Closest Point
  if (CurrentPoint < PathPoints.size() - 1) PWLθ = atan2(-1 * (ClosestPoint.GetX() - PathPoints.at(CurrentPoint + 1).GetX()), ClosestPoint.GetY() - PathPoints.at(CurrentPoint + 1).GetY()) * -1;
  // Wrap the target heading around 360 (0 - 360)
  if (PWLθ < 0) PWLθ += (2 * M_PI);

  // Angle of the Vector of the First LookAhead Point (P_D) to the New LookAhead Point (P_L) - Since we don't have the new lookahead point
  // the angle is based of the fact that the vector forms a 90° corner with the PWD Vector meaning me either add or subtract 90° depending on which side its on.
  PDLθ = PWDθ - (Sign(PWLθ - PWDθ) * M_PI) / 2;

  /* Calculating Degree*/
  DisToClosestPointFraction = sqrt(pow(MCL::X - ClosestPoint.GetX(), 2) + pow(MCL::Y - ClosestPoint.GetY(), 2)) / 3; //Instead of trackwidth try 1, 3, and 5 // Probbaby between 2 and 4

  if (DisToClosestPointFraction > 1) DisToClosestPointFraction = 1; // If decided to keep a running max dis to the closest point than this isnt needed

  if (CurrentPoint + LookAheadDis/DisBetweenPoint < PathPoints.size() - 1) LookAheadCurvature = PathPoints.at(CurrentPoint + LookAheadDis/DisBetweenPoint).GetCurve();

  if (ClosestPoint.GetCurve() >= LookAheadCurvature) {
    CurvatureDifference = 0;
  } else if (LookAheadCurvature - ClosestPoint.GetCurve() < MaxCurve) {
    CurvatureDifference = (LookAheadCurvature - ClosestPoint.GetCurve()) / MaxCurve;
  } else if (LookAheadCurvature - ClosestPoint.GetCurve() >= MaxCurve) {
    CurvatureDifference = 1;
  }

  Degree = (1 - DisToClosestPointFraction) * CurvatureDifference;
  
  /* Making the new Point */
  return Point(FirstLookAheadPoint.GetX() + (Degree * (sqrt(pow(ClosestPoint.GetX() - FirstLookAheadPoint.GetX(), 2) + pow(ClosestPoint.GetY() - FirstLookAheadPoint.GetY(), 2))) * tan(fabs(PWLθ - PWDθ)) * cos(PDLθ)), FirstLookAheadPoint.GetY() + (Degree * (sqrt(pow(ClosestPoint.GetX() - FirstLookAheadPoint.GetX(), 2) + pow(ClosestPoint.GetY() - FirstLookAheadPoint.GetY(), 2))) * tan(fabs(PWLθ - PWDθ)) * sin(PDLθ)));
}

double PurePursuit::GetLookAheadDis(double PrevLookAheadDis, double CurrentVeclocity, Point Close) {
  // CurrentVeclocity - Probably average of left and right could be something else though like highest or lowest
  double minLookaheadDistance = 12, maxLookaheadDistance = 25, k_speed = .16, k_curvature = 12; // K curvature between 10-30
  return max(minLookaheadDistance, min(maxLookaheadDistance, (PrevLookAheadDis - k_curvature * fabs(Close.GetCurve())) + (2 - (k_speed * CurrentVeclocity * 2)))); // My way
  // double minLookaheadDistance = 12, maxLookaheadDistance = 25, k_speed = 2.67;
  // return max(minLookaheadDistance, min(maxLookaheadDistance, k_speed * CurrentVeclocity)); // Only to Speed

  // double minLookaheadDistance = 12, maxLookaheadDistance = 25, k_curvature = 72;
  // return max(minLookaheadDistance, min(maxLookaheadDistance, (k_curvature * fabs(Close.GetCurve())))); // Only to Curvature
}

double PurePursuit::GetLookAheadDis(double BaseLookAheadDis, Point Close, Point Next) {
  // Vector from Current Closest Point to the Current Position
  Point W = Point(MCL::X - Close.GetX(), MCL::Y - Close.GetY());
  // Vector from Current Closest Point to the Next Closest Point
  Point V = Point(Next.GetX() - Close.GetX(), Next.GetY() - Close.GetY());
  // stuff and tings
  B = ((W.GetX() * V.GetX()) + (W.GetY() * V.GetY())) / ((V.GetX() * V.GetX()) + (V.GetY() * V.GetY()));
  // Making the point ig
  Point ProjectionVector = Point(Close.GetX() + (B * V.GetX()), Close.GetY() + (B * V.GetY()));
  // Adding the dis to path for scoring
  // DisToPath.push_back(sqrt(pow(MCL::X - ProjectionVector.GetX(), 2) + pow(MCL::Y - ProjectionVector.GetY(), 2)));
  // Return the updated lookahead
  return BaseLookAheadDis + sqrt(pow(MCL::X - ProjectionVector.GetX(), 2) + pow(MCL::Y - ProjectionVector.GetY(), 2));
}

void PurePursuit::GetAccuracy(void) {
  double Max = 0, Total = 0;
  /* Horizontal Dis to Closest Point */
  for (int Grade = 0; Grade < Score.size(); Grade++) {
    if (Score.at(Grade) > Max) {Max = Score.at(Grade);}
    Total += Score.at(Grade);
  }
  cout << "Horizontal Dis To Closest Point" << "\tTrackWidth / 2:: " << 8 << endl;
  // Max
  cout << "Min:: " << Max << "\t" << ((8 - Max) / 8) * 100 << endl;
  // Average
  cout << "Average:: " << Total/Score.size() << "\t" << ((8 - (Total/Score.size())) / 8) * 100 << endl;

  /* Dis to Closest Point */
  Max = 0, Total = 0;
  for (int Grade = 0; Grade < ClosestPointError.size(); Grade++) {
    if (ClosestPointError.at(Grade) > Max) {Max = ClosestPointError.at(Grade);}
    Total += ClosestPointError.at(Grade);
  }
  cout << "Dis To Closest Point" << "\tTrackWidth / 2:: " << 8 << endl;
  // Max
  cout << "Min:: " << Max << "\t" << ((8 - Max) / 8) * 100 << endl;
  // Average
  cout << "Average:: " << Total/ClosestPointError.size() << "\t" << ((8 - (Total/ClosestPointError.size())) / 8) * 100 << endl;

  /* Dis to Path */
  Max = 0, Total = 0;
  for (int Grade = 0; Grade < DisToPath.size(); Grade++) {
    if (DisToPath.at(Grade) > Max) {Max = DisToPath.at(Grade);}
    Total += DisToPath.at(Grade);
  }
  
  cout << "Dis To Closest Point" << "\tTrackWidth / 2:: " << 8 << endl;
  // Max
  cout << "Min:: " << Max << "\t" << ((8 - Max) / 8) * 100 << endl;
  // Average
  cout << "Average:: " << Total/DisToPath.size() << "\t" << ((8 - (Total/DisToPath.size())) / 8) * 100 << endl;
  cout << "\n\n";
  // return ((8 - (Total/DisToPath.size())) / 8) * 100;
}

void PurePursuit::ClearPath(void) {
  IsClearing = true;
  // Clearing the Paths
  if (!PathPoints.empty()) {
    PathPoints.clear();
  }
  // Clearing the Vars
  DisBetweenPoint = 0;
  RelativeDis = 0;
  AmountOfPoints = 0;
  X1 = 0, X2 = 0, X3 = 0, Y1 = 0, Y2 = 0, Y3 = 0;
  K1 = 0, K2 = 0, B = 0, A = 0, Radius = 0, Curvature = 0;
  NewVel = 0;
  CurrentPoint = 0;
  Loop = 1;
  TempDis = 225;
  DistanceToPoint = 0;
  a = 0, b = 0, c = 0, Discriminant = 0;
  T_Intersection = 0, T1 = 0, T2 = 0;
  FractionalIndex = 0, PrevFractionalIndex = 0;
  LookAheadPointFound = false;
  ArcCurvature = 0;
  StandardA = 0, StandardB = 0, StandardC = 0;
  HorizontalDisToPoint = 0, Side = 0;
  TargetVelocity = 0;
  LeftTargetVelocity = 0;
  RightTargetVelocity = 0;
  CurveOfArc = 0; 
  LeftFeedForward = 0, RightFeedForward = 0;
  LeftTargetAccel = 0, RightTargetAccel = 0, PrevLeftTargetVelocity = 0, PrevRightTargetVelocity = 0;
  LeftMeasuredVelocity = 0, RightMeasuredVelocity = 0;
  LeftFeedBack = 0, RightFeedBack = 0;
  LeftPower = 0, RightPower = 0;
  IsClearing = false;
}





// Main Namespace
namespace Boom {
  double TargetX, TargetY, TargetAngle;
  double MaxVelo = 12, MinVelo = 0, Fraction;
  double CarrotPointX, CarrotPointY;
  double Dis, Ang, CurveRatio = .5; // Can Mess With
  double AngularError, LinSpeed;
  double LinearKP = .36, AngluarKP = .12; // Still Needs to be Tuned // Tune by going straight making it as fast as possible then add the angular stuff
  double OverTurn, NewAngularError;
  double LeftPower, RightPower;
  double AngularSpeed, Tolerance = 6, Duration = 2000, TimeoutDis = 12;
  // Indicates End Angle or No End Angle
  bool EndPose = true;
  bool CanReverse = false;
  bool LeTimeoutState = true;
  // Timer
  timer Timeout;
  // Start Boomerang Variable
  bool StartBoomerang = false;
  // Functions
  void StopBoom(void);
  void MoveToPoint(int EndX, int EndY, double Max, double Min, bool Reverse);
  void MoveToPose(int EndX, int EndY, double EndAngle, double Max, double Min, bool Reverse);
  // Main Loop
  void Boomerang(void);


  double CalcTurnAngle(double TurnTargetX, double TurnTargetY) {
    // Get the angle from the rotated graph
    TargetAngle = atan2((TurnTargetX - MCL::X) * -1, TurnTargetY - MCL::Y) * -1;
    // Wrap the target heading around 360 (0 - 360)
    if (TargetAngle < 0) TargetAngle += (2 * M_PI);
    // Use the angle to get the global target heading
    double Amount = (TargetAngle - AngleWrap(MCL::theta));
    // Wrap the Turning angle around 180
    if (Amount > M_PI) {
      Amount -= 2 * M_PI;
    } else if (Amount < -M_PI) {
      Amount += 2 * M_PI;
    }
    // Return the Angle in Radians
    return Amount;
  }
}

void Boom::MoveToPoint(int EndX, int EndY, double Max, double Min, bool Reverse) {
  // Setting the End Point
  TargetX = EndX;
  TargetY = EndY;
  // Setting the Timeout
  LeTimeoutState = false;
  // Setting No End Angle
  EndPose = false, CanReverse = Reverse;
  // Setting the Maximum/Minimum Speeds
  MaxVelo = Max, MinVelo = Min;
  // Start Boomerang
  StartBoomerang = true;
}

void Boom::MoveToPose(int EndX, int EndY, double EndAngle, double Max, double Min, bool Reverse) {
  // Setting the End Point
  TargetX = EndX;
  TargetY = EndY;
  TargetAngle = EndAngle;
  // Setting the Timeout
  LeTimeoutState = false;
  // Setting No End Angle
  EndPose = true, CanReverse = Reverse;
  // Setting the Maximum Speed
  MaxVelo = Max, MinVelo = Min;
  // Start Boomerang
  StartBoomerang = true;
}

void Boom::Boomerang(void) {
  if (StartBoomerang) {
  /* Calculating the Carrot Point */
    // Distance to the Target Point
    Dis = sqrt(pow(TargetX - MCL::X, 2) + pow(TargetY - MCL::Y, 2));

    // Behavior depending on whether or not EndAngle
    if (!EndPose || Dis < Tolerance) {
      // The "Carrot Point" is just the Target Point
      CarrotPointX = TargetX;
      CarrotPointY = TargetY;
    } else {
      // Calculate the "CarrotPoint" to curve around into the EndAngle
      Ang = ToRadians(90 - TargetAngle);

      // Making the Point
      CarrotPointX = TargetX - (Dis * cos(Ang) * CurveRatio);
      CarrotPointY = TargetY - (Dis * sin(Ang) * CurveRatio);
    }

    // Calcualting the Angular Error
    AngularError = CalcTurnAngle(CarrotPointX, CarrotPointY);

    // Setting the Linear Speed
    LinSpeed = Dis * LinearKP;

    // Limiting the Speed
    if (LinSpeed > MaxVelo) LinSpeed = MaxVelo;
    if (LinSpeed < MinVelo) LinSpeed = MinVelo;

    // Checking timeout
    // if ((Dis + 8) < TimeoutDis && !TimeoutState) {
    //   Timeout.reset();
    //   TimeoutState = true; 
    // }

  /* Setting the Angular Speed */
    if (Dis < Tolerance) {

      // Letting the Robot Oscillate
      CanReverse = true;

      if (!EndPose) {
        AngularSpeed = 0;
      } else {
        NewAngularError = ToRadians(TargetAngle) - MCL::theta;
        while (fabs(NewAngularError) > M_PI) {
          NewAngularError -= 2 * M_PI * Sign(NewAngularError);
        }

        AngularSpeed = ToDegrees(NewAngularError) * AngluarKP;

        if (fabs(AngularSpeed) < MinVelo) {
          AngularSpeed = Sign(AngularSpeed) * MinVelo;
        }
      }

      LinSpeed *= cos(AngularError);
    } else {
      // Check This
      if (fabs(AngularError) > M_PI_2 && CanReverse) {
        AngularError -= Sign(AngularError) * M_PI;
        LinSpeed = -LinSpeed;
      }

      AngularSpeed = ToDegrees(AngularError) * AngluarKP;
    }

    // Overturn
    OverTurn = fabs(AngularSpeed) + fabs(LinSpeed) - MaxVelo;
    if (OverTurn > 0) LinSpeed -= Sign(LinSpeed) * OverTurn;

    LeftPower = LinSpeed + AngularSpeed;
	  RightPower = LinSpeed - AngularSpeed;

    // Better way of limiting power
    // Fraction = max(fabs(LeftPower), fabs(RightPower)) / MaxVelo;
    // // Fraction'ing the Powers
    // if (Fraction > 1) LeftPower /= Fraction, RightPower /= Fraction;

    // Fraction = min(fabs(LeftPower), fabs(RightPower)) / MinVelo;
    // // Fraction'ing the Powers
    // if (Fraction < 1) LeftPower /= Fraction, RightPower /= Fraction;



    //                                  Mess around with Dis <= 1 or 2 since Carrot becomes Target
    if ((Dis <= 4 && !EndPose) || (Dis <= 3 && ToDegrees(NewAngularError) <= 1 && ToDegrees(NewAngularError) >= -1 && EndPose) || LeTimeoutState == true) {
      L1.stop(brake);
      L2.stop(brake);
      L3.stop(brake);
      R1.stop(brake);
      R2.stop(brake);
      R3.stop(brake);
      StartBoomerang = false;
      this_thread::yield();
    } else {
      L1.spin(fwd, LeftPower, volt);
      L2.spin(fwd, LeftPower, volt);
      L3.spin(fwd, LeftPower, volt);
      R1.spin(fwd, RightPower, volt);
      R2.spin(fwd, RightPower, volt);
      R3.spin(fwd, RightPower, volt);
    }
  }
}

void Boom::StopBoom(void) {
  LeTimeoutState = true;
}

// #include "cmath"
// #include "vex.h"
// #include "api.h"

// using namespace std;

// namespace Ramsete {
//   // --- Parameters ---
//   // Tuning Gains (Adjust these based on testing)
//   double lmao = 5;
//   const double dd = .1;
//   const double b = .02;     // Aggressiveness > 0 (higher values make convergence faster)
//   const double zeta = .07;  // Damping ratio (0 < zeta < 1) (higher values reduce oscillation)
//   double trackWidth = 13.5; // Inches - MAKE SURE THIS MATCHES CONSTRAINTS
//   const double MAX_SPEED = 76.5; // Example: Max speed your robot can achieve in in/s
//   const double MAX_VOLTAGE = 12.0;

//   // Control State
//   bool isFollowing = false;
//   int currentTrajectoryIndex = 0;
//   vector<ProfilePoint> currentTrajectory;


//   /* Helper Function */
//   double sinc(double x) {
//     if (abs(x) < 1e-9) {
//       return 1.0;
//     } else {
//       return sin(x) / x;
//     }
//   }

//   // You might need Feedforward (kV, kA, kS) and/or PID control here.
//   double wheelSpeedToVoltage(double wheelSpeed_ips) {
//     double volt = (wheelSpeed_ips / MAX_SPEED) * MAX_VOLTAGE;
//     if (volt > MAX_VOLTAGE) volt = MAX_VOLTAGE;
//     if (volt < -MAX_VOLTAGE) volt = -MAX_VOLTAGE;

//     return volt;
//     // return wheelSpeed_ips;
//   }


//   // --- Main Control Task ---

//   // Call this once to load the trajectory and start the control loop task
//   void startFollowing(const vector<ProfilePoint>& trajectoryToFollow) {
//     if (trajectoryToFollow.empty()) {
//       // Handle empty trajectory case
//       cout << "Error: Cannot follow an empty trajectory." << endl;
//       return;
//     }
//     currentTrajectory = trajectoryToFollow;
//     currentTrajectoryIndex = 1;
//     isFollowing = true;
//     cout << "Ramsete Following Started. Trajectory size: " << currentTrajectory.size() << endl;
//   }


//   // Main Function
//   void RamseteLoop(void) {
//     while (true) {
//       if (isFollowing) { 
//         // Get Goal State from Trajectory
//         const ProfilePoint& goal = currentTrajectory[currentTrajectoryIndex];
//         const double vd = goal.vel;     // Desired linear velocity (in/s)
//         const double wd = goal.omega;   // Desired angular velocity (rad/s)

//         const double dt = dd / fabs(vd);

//         // Get Current Robot Pose from MCL
//         const double currentX = MCL::X;
//         const double currentY = MCL::Y;
//         const double currentTheta = M_PI_2 - MCL::theta; // Rotated because we are weird 😛

//         // Calculate Errors
//         const double exg = goal.x - currentX;
//         const double eyg = goal.y - currentY;

//         // const double exg = 0;
//         // const double eyg = 0;

//         const double ex = exg * cos(currentTheta) + eyg * sin(currentTheta);
//         const double ey = -exg * sin(currentTheta) + eyg * cos(currentTheta); // const double ey = exg * sin(currentTheta) - eyg * cos(currentTheta);
//         const double etheta = Normalize(goal.theta - currentTheta);
//         // const double etheta = 0;
        

//         // Calculate Ramsete Gain (k)
//         const double k = 2.0 * zeta * sqrt(wd * wd + b * vd * vd);

//         // Calculate Commanded Velocities (v, omega) using Ramsete equations
//         const double linearVel = vd * cos(etheta) + k * ex;
//         const double angularVel = wd + k * etheta + b * vd * sinc(etheta) * ey;

//         // Convert Chassis Speeds to Wheel Speeds
//         const double leftSpeed = linearVel - (angularVel * trackWidth / 2.0);
//         const double rightSpeed = linearVel + (angularVel * trackWidth / 2.0);

//         // const double leftSpeed = vd - (wd * trackWidth / 2.0);
//         // const double rightSpeed = vd + (wd * trackWidth / 2.0);

//         // Convert Wheel Speeds to Motor Commands (Voltage, RPM, etc.)
//         double LeftPower = wheelSpeedToVoltage(leftSpeed);
//         double RightPower = wheelSpeedToVoltage(rightSpeed);

//         double Fraction = max(fabs(LeftPower), fabs(RightPower)) / MAX_VOLTAGE;
//         // // Fraction'ing the Powers
//         if (Fraction > 1) LeftPower /= Fraction, RightPower /= Fraction;
        
//         // Fraction = min(fabs(LeftPower), fabs(RightPower)) / MinSpeed;
//         // Fraction'ing the Powers
//         // if (Fraction < 1) LeftPower /= Fraction, RightPower /= Fraction;

//         cout << LeftPower << "    " << RightPower << endl;

//         if (currentTrajectoryIndex >= currentTrajectory.size() - 1) {
//           // Reached the end of the trajectory
//           L1.stop(brake);
//           L2.stop(brake);
//           L3.stop(brake);
//           R1.stop(brake);
//           R2.stop(brake);
//           R3.stop(brake);
//           isFollowing = false;
//           currentTrajectoryIndex = 1;
//         } else {
//           // Left Side
//           L1.spin(fwd, LeftPower, volt);
//           L2.spin(fwd, LeftPower, volt);
//           L3.spin(fwd, LeftPower, volt);
//           // Right Side
//           R1.spin(fwd, RightPower, volt);
//           R2.spin(fwd, RightPower, volt);
//           R3.spin(fwd, RightPower, volt);
//         }

//         currentTrajectoryIndex++;

//         // lmao = dt * 1000;
//         // wait(dt * 1000, msec);
//       }

//       wait(5, msec);
//     }
//   }




// }