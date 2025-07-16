// #pragma once

// #include "Misc/Matrix.h"
// #include "Misc/Point.h"

// class CatmullRom {
//   public:
//   /* Constructors */
//     /** 
//     * @brief Creates an instance of a Catmull Rom Object
//     */
//     CatmullRom() {}

//     // Default Empty Destructor
//     ~CatmullRom() {}

//     // Default Non-Empty Constructor
    // CatmullRom(std::vector<Point> controlPoints) {
    //   /* Save the Number of Segments */
    //   this->segNums = controlPoints.size() - 1;
    //   // cout << segNums << endl;

    //   /* Create Ghost Points and Add the Control Poits*/
    //   std::vector<Point> newPoints;
    //   // First Ghost Point
    //   newPoints.push_back(controlPoints[0] - (controlPoints[1] - controlPoints[0]));
    //   // Adding Normal Points
    //   for (int s = 0; s < controlPoints.size(); s++) {
    //     newPoints.push_back(controlPoints[s]);
    //   }
    //   // Last Ghost Point
    //   newPoints.push_back(controlPoints.back() - (controlPoints[controlPoints.size() - 2] - controlPoints.back()));

    //   // Setting the New Control Points
    //   this->splinePoints = newPoints;

    //   // Setting the Arc Length
    //   double dt = .01;
    //   for (double x = dt; x < segNums; x += dt) {
    //     this->arcLength += interpolate(x).distToPoint(interpolate(x - dt));
    //     this->moreSegNums += 1;
    //   }
    // }

//     // CatmullRom(CatmullRom *Path) {
//     //   *this = *Path;
//     // }

//   /* Functions */
//     double get_t_at_dist(double dist) {
//       return dist/arcLength;
//     }

//     Point interpolate(double t) {
//       if (t < 0 || t > segNums) {throw std::logic_error("Invalid T-Value for Spline");}
//       // Getting the Segment Number
//       int Segment = floor(t);
//       // Getting the Matrix of Control Points given the Segment
//       Matrix PointMatrix = getControlPoints(Segment);
//       // Get the t value of the Segment
//       t -= Segment;
//       // Co-effiecent Matrix
//       Matrix Coeff = Matrix({{1, t, t * t, t * t * t}});
//       // Resultant Matrix
//       Matrix R = Coeff * Characteristic * PointMatrix;
//       // Return
//       return Point(R(0, 0), R(0, 1), getCurvature(t));
//     }

//     // Returns the Velocity as a vector of X and Y
//     Point getVel(double t) {
//       if (t < 0 || t > segNums) {throw std::logic_error("Invalid T-Value for Spline");}
//       // Getting the Segment Number
//       int Segment = (int)(t);
//       cout << Segment << endl;
//       // Getting the Matrix of Control Points given the Segment
//       Matrix PointMatrix = getControlPoints(Segment);
//       // Get the t value of the Segment
//       t -= Segment;
//       // Co-effiecent Matrix
//       Matrix Coeff = Matrix({{0, 1, 2 * t, 3 * t * t}});
//       // Resultant Matrix
//       Matrix R = Coeff * Characteristic * PointMatrix;
//       // Return
//       return Point(R(0, 0), R(0, 1));
//     }

//     // Returns the Accleration as a vector of X and Y
//     Point getAccel(double t) {
//       if (t < 0 || t > segNums) {throw std::logic_error("Invalid T-Value for Spline");}
//       // Getting the Segment Number
//       int Segment = floor(t);
//       // Getting the Matrix of Control Points given the Segment
//       Matrix PointMatrix = getControlPoints(Segment);
//       // Get the t value of the Segment
//       t -= Segment;
//       // Co-effiecent Matrix
//       Matrix Coeff = Matrix({{0, 0, 2, 6 * t}});
//       // Resultant Matrix
//       Matrix R = Coeff * Characteristic * PointMatrix;
//       // Return
//       return Point(R(0, 0), R(0, 1));
      
//     }

//     double getCurvature(double t) {
//       // Get Velocity
//       Point dx = this->getVel(t);
//       // Get Acceleration
//       Point ddx = this->getAccel(t);
//       // Find the Numerator
//       double numerator = dx.GetX() * ddx.GetY() - dx.GetY() * ddx.GetX();
//       // Find the Denominator
//       double denominator = pow(dx.GetX() * dx.GetX() + dx.GetY() * dx.GetY(), 1.5);

//       // Return the Curvature
//       return abs(numerator) / denominator;
//     }

//     double getCurvature(Point dx, Point ddx) {
//       // Find the Numerator
//       double numerator = dx.GetX() * ddx.GetY() - dx.GetY() * ddx.GetX();
//       // Find the Denominator
//       double denominator = pow(dx.GetX() * dx.GetX() + dx.GetY() * dx.GetY(), 1.5);

//       // Return the Curvature
//       return abs(numerator) / denominator;
//     }

//     double getYaw(double t) {
//       Point dx = this->getVel(t);
//       return atan2(dx.GetX(), dx.GetY());
//     }

//     double getSegments() {
//       return segNums;
//     }

//     double getSmallSegments() {
//       return moreSegNums;
//     }

//     double getLength() {
//       return arcLength;
//     }

//   private:
//     /* Instance Data */
//     // Values
//     int segNums = 0;
//     int moreSegNums = 0;

//     // dt = 50 Miliseconds, dd = 1 inch
//     double arcLength = 0;

//     // All Necessary Points
//     std::vector<Point> splinePoints;

//     // Characteristic Matrix
//     Matrix Characteristic = Matrix({
//       {  0,    1,    0,   0},
//       {-.5,    0,   .5,   0},
//       {1.0, -2.5,  2.0, -.5},
//       {-.5,  1.5, -1.5,  .5},
//     });

//     /* Functions */
//     Matrix getControlPoints(int Segment) {
//       Point P0 = splinePoints[Segment], P1 = splinePoints[Segment + 1], P2 = splinePoints[Segment + 2], P3 = splinePoints[Segment + 3];
//       return Matrix({{P0.GetX(), P0.GetY()}, {P1.GetX(), P1.GetY()}, {P2.GetX(), P2.GetY()}, {P3.GetX(), P3.GetY()}});
//     }    
//   };
// /*
//   class CatmullRomSpline : public virtualPath {
// public:
//     CatmullRomSpline(const std::vector<Point2D>& controlPoints) {
//         if (controlPoints.size() < 2) {
//             throw std::invalid_argument("At least 2 control points required");
//         }

//         // Create ghost points for end conditions
//         points.reserve(controlPoints.size() + 2);
//         points.push_back(controlPoints[0] * 2 - controlPoints[1]);  // Ghost start point
//         points.insert(points.end(), controlPoints.begin(), controlPoints.end());
//         points.push_back(controlPoints.back() * 2 - controlPoints[controlPoints.end() - 2]);  // Ghost end point

//         segmentCount = controlPoints.size() - 1;
//         precomputeLength();
//     }

//     Point2D getPoint(double t) override {
//         const auto [segmentIdx, localT] = normalizeT(t);
//         Matrix coeff({{1, localT, localT * localT, localT * localT * localT}});
//         Matrix pointMatrix = getSegmentPoints(segmentIdx);
//         Matrix result = coeff * BASIS_MATRIX * pointMatrix;
//         return Point2D(result(0, 0), result(0, 1));
//     }

//     Point2D getDerivative(double t) override {
//         const auto [segmentIdx, localT] = normalizeT(t);
//         Matrix coeff({{0, 1, 2 * localT, 3 * localT * localT}});
//         Matrix pointMatrix = getSegmentPoints(segmentIdx);
//         Matrix result = coeff * BASIS_MATRIX * pointMatrix;
//         return Point2D(result(0, 0), result(0, 1));
//     }

//     Point2D getSecondDerivative(double t) override {
//         const auto [segmentIdx, localT] = normalizeT(t);
//         Matrix coeff({{0, 0, 2, 6 * localT}});
//         Matrix pointMatrix = getSegmentPoints(segmentIdx);
//         Matrix result = coeff * BASIS_MATRIX * pointMatrix;
//         return Point2D(result(0, 0), result(0, 1));
//     }

//     double getCurvature(double t) override {
//         Point2D d = getDerivative(t);
//         Point2D dd = getSecondDerivative(t);
//         return getCurvature(d, dd);
//     }

//     double getCurvature(Point2D d, Point2D dd) override {
//         return (d.x * dd.y - d.y * dd.x) / pow(d.x * d.x + d.y * d.y, 1.5);
//     }

//     double get_t_at_arc_length(double arcLength) override {
//         if (arcLength <= 0) return 0;
//         if (arcLength >= totalLength) return 1;

//         // Binary search through the precomputed lengths
//         size_t left = 0;
//         size_t right = arcLengths.size() - 1;
        
//         while (left < right - 1) {
//             size_t mid = (left + right) / 2;
//             if (arcLengths[mid] < arcLength) {
//                 left = mid;
//             } else {
//                 right = mid;
//             }
//         }

//         // Linear interpolation between samples
//         double t0 = left * SAMPLING_STEP;
//         double t1 = right * SAMPLING_STEP;
//         double s0 = arcLengths[left];
//         double s1 = arcLengths[right];
        
//         return t0 + (t1 - t0) * (arcLength - s0) / (s1 - s0);
//     }

//     double getLength() override {
//         return totalLength;
//     }

// private:
//     static constexpr double SAMPLING_STEP = 0.001;  // Higher resolution for better accuracy
//     static const Matrix BASIS_MATRIX;  // Catmull-Rom basis matrix

//     std::vector<Point2D> points;
//     std::vector<double> arcLengths;
//     int segmentCount;
//     double totalLength = 0;

//     std::pair<int, double> normalizeT(double t) const {
//         if (t < 0 || t > 1) {
//             throw std::out_of_range("Parameter t must be between 0 and 1");
//         }
//         double scaledT = t * segmentCount;
//         int segmentIdx = std::min(static_cast<int>(scaledT), segmentCount - 1);
//         double localT = scaledT - segmentIdx;
//         return {segmentIdx, localT};
//     }

//     Matrix getSegmentPoints(int segmentIdx) const {
//         return Matrix({
//             {points[segmentIdx].x, points[segmentIdx].y},
//             {points[segmentIdx + 1].x, points[segmentIdx + 1].y},
//             {points[segmentIdx + 2].x, points[segmentIdx + 2].y},
//             {points[segmentIdx + 3].x, points[segmentIdx + 3].y}
//         });
//     }

//     void precomputeLength() {
//         const size_t numSamples = static_cast<size_t>(1.0 / SAMPLING_STEP) + 1;
//         arcLengths.reserve(numSamples);
//         arcLengths.push_back(0);

//         Point2D prevPoint = getPoint(0);
//         double length = 0;

//         for (size_t i = 1; i < numSamples; ++i) {
//             double t = i * SAMPLING_STEP;
//             Point2D currentPoint = getPoint(t);
            
//             double dx = currentPoint.x - prevPoint.x;
//             double dy = currentPoint.y - prevPoint.y;
//             length += sqrt(dx * dx + dy * dy);
            
//             arcLengths.push_back(length);
//             prevPoint = currentPoint;
//         }

//         totalLength = length;
//     }
// };

// // Initialize the static basis matrix
// const Matrix CatmullRomSpline::BASIS_MATRIX = Matrix({
//     { 0,  1,  0,  0},
//     {-0.5, 0,  0.5, 0},
//     { 1, -2.5, 2, -0.5},
//     {-0.5, 1.5,-1.5, 0.5}
// });


// */


// /* 
// class Cardinal {
//   public:
//     // Default Empty Constructor
//     Cardinal() {}

//     // Default Empty Destructor
//     ~Cardinal() {}

//     // Default Non-Empty Constructor
//     Cardinal(std::vector<Point> controlPoints, double s = 1) {
//       this->S = s;
//       Save the Number of Segments
//       this->segNums = controlPoints.size() - 1;
//       Create Ghost Points
//       std::vector<Point> newPoints;
//       // First Point
//       newPoints.push_back(controlPoints[0] - (controlPoints[1] - controlPoints[0]));
//       // Normal Points
//       for (int s = 0; s < controlPoints.size(); s++) {
//         newPoints.push_back(controlPoints[s]);
//       }
//       // Last Point
//       newPoints.push_back(controlPoints.back() - (controlPoints[controlPoints.size() - 2] - controlPoints.back()));

//       // Setting the Control Points
//       this->pathPoints = newPoints;
//     }

//     Point interpolate(double t) {
//       if (t < 0 || t > segNums) {
//         return Point (0, 0);
//       }
//       // Comment Here
//       int Segment = floor(t);

//       // Turn this into a function
//       Matrix PointMatrix;
//       if (Segment <= segNums)
//         PointMatrix = getControlPoints(Segment);

//       t -= Segment;


//       // Co-effiecent Matrix
//       Matrix Coeff = Matrix({{1, t, t * t, t * t * t}});

//       // Resultant Matrix
//       Matrix R = Coeff * Characteristic * PointMatrix;

//       return Point(R(0, 0), R(0, 1));
//     }

//     Point getVel(double t) {
//       int currentSegment = floor(t);
//       t -= currentSegment;

//       Matrix pointMatrix = getControlPoints(currentSegment);
//       Matrix Coeff = Matrix({{0, 1, 2 * t, 3 * t * t}});

//       Matrix result = Coeff * Characteristic * pointMatrix;

//       return Point(result(0, 0), result(0, 1));
//     }

//     Point getAccel(double t) {
//       int currentSegment = floor(t);
//       t -= currentSegment;

//       Matrix pointMatrix = getControlPoints(currentSegment);
//       Matrix Coeff = Matrix({{0, 0, 2, 6 * t}});

//       Matrix result = Coeff * Characteristic * pointMatrix;

//       return Point(result(0, 0), result(0, 1));
//     }

//     double curvature(double t) {
//       Point d = getVel(t);
//       Point dd = getAccel(t);

//       double numerator = d.GetX() * dd.GetY() - d.GetY() * dd.GetX();
//       double denominator = pow(d.GetX() * d.GetX() + d.GetY() * d.GetY(), 1.5);

//       return abs(numerator) / denominator;
//     }

//     double getYaw(double t) {
//       Point dx = getVel(t);
//       return atan2(dx.GetX(), dx.GetY());
//     }

//     int getSegments() {
//       return segNums;
//     }

//   private:
//     Instance Data
//     // Nums
//     int segNums;
//     // Scale
//     double S = 0;

//     // All Necessary Points
//     std::vector<Point> pathPoints;

//     // Characteristic Matrix
//     Matrix Characteristic = Matrix({
//     {         0,       1,          0,   0},
//     {        -S,       0,          S,   0},
//     {     2 * S, S - 2.5,  3 - 2 * S,  -S},
//     {        -S,   2 - S,      S - 2,   S},
//     });

//     Functions
//     Matrix getControlPoints(int Segment) {
//       Point P0 = pathPoints[Segment], P1 = pathPoints[1 + Segment], P2 = pathPoints[2 + Segment], P3 = pathPoints[3 + Segment];
//       return Matrix({{P0.GetX(), P0.GetY()}, {P1.GetX(), P1.GetY()}, {P2.GetX(), P2.GetY()}, {P3.GetX(), P3.GetY()}});
//     }    
//   };


//   class Hermite {
//   public:
//     // Default Empty Constructor
//     Hermite() {}

//     // Default Empty Destructor
//     ~Hermite() {}

//     // Default Non-Empty Constructor
//     Hermite(std::vector<Point> controlPoints) {
//       // Setting the Control Points
//       this->pathPoints = controlPoints;
//     }

//     Point interpolate(double t) {
//       if (t < 0 || t > segNums) {
//         return Point (0, 0);
//       }
//       // Comment Here
//       int Segment = floor(t);

//       // Turn this into a function
//       Matrix PointMatrix;
//       if (Segment <= segNums)
//         PointMatrix = getControlPoints(Segment);

//       t -= Segment;


//       // Co-effiecent Matrix
//       Matrix Coeff = Matrix({{1, t, t * t, t * t * t}});

//       // Resultant Matrix
//       Matrix R = Coeff * Characteristic * PointMatrix;

//       return Point(R(0, 0), R(0, 1));
//     }

//     Point getVel(double t) {
//       int currentSegment = floor(t);
//       t -= currentSegment;

//       Matrix pointMatrix = getControlPoints(currentSegment);
//       Matrix Coeff = Matrix({{0, 1, 2 * t, 3 * t * t}});

//       Matrix result = Coeff * Characteristic * pointMatrix;

//       return Point(result(0, 0), result(0, 1));
//     }

//     Point getAccel(double t) {
//       int currentSegment = floor(t);
//       t -= currentSegment;

//       Matrix pointMatrix = getControlPoints(currentSegment);
//       Matrix Coeff = Matrix({{0, 0, 2, 6 * t}});

//       Matrix result = Coeff * Characteristic * pointMatrix;

//       return Point(result(0, 0), result(0, 1));
//     }

//     double curvature(double t) {
//       Point d = getVel(t);
//       Point dd = getAccel(t);

//       double numerator = d.GetX() * dd.GetY() - d.GetY() * dd.GetX();
//       double denominator = pow(d.GetX() * d.GetX() + d.GetY() * d.GetY(), 1.5);

//       return abs(numerator) / denominator;
//     }

//     double getYaw(double t) {
//       Point dx = getVel(t);
//       return atan2(dx.GetX(), dx.GetY());
//     }

//     int getSegments() {
//       return segNums;
//     }

//   private:
//     Instance Data
//     // Nums
//     int segNums;

//     // All Necessary Points
//     std::vector<Point> pathPoints;

//     // Characteristic Matrix
//     Matrix Characteristic = Matrix({
//     {  1,   0,   0,  0},
//     {  0,   1,   0,  0},
//     { -3,  -2,   3, -1},
//     {  2,   1,  -2,  1},
//     });

//     Functions
//     Matrix getControlPoints(int Segment) {
//       Point P1 = pathPoints[Segment], P2 = pathPoints[Segment + 1];
//       Point V1 = pathPoints[Segment], V2 = pathPoints[Segment + 1].GetVelocity();
//       return Matrix({{P0.GetX(), P0.GetY()}, {P1.GetX(), P1.GetY()}, {P2.GetX(), P2.GetY()}, {P3.GetX(), P3.GetY()}});
//     }
//   }; */















































//   // #pragma once
// // #include "Misc/Spline.h"
// // // #include "Misc/Kinematics.h"
// // #include "Misc/Classes/State.h"

// // /* Maybe turn into a namespace */
// // using namespace State;


// // class Trajectory {
// //   public:
// //   /* Constructors */
// //     /** 
// //     * @brief Creates an instance of a Trajectory Rom Object
// //     */
// //     Trajectory() {}

// //     // Default Empty Destructor
// //     ~Trajectory() {}


// //     Trajectory(std::vector<Point> controlPoints, double accelDis = 12, double dt = .05, double dd = .1) {
// //       this->Path = CatmullRom(controlPoints);
      
// //       this->accelDis = accelDis; // In Inches

// //       this->maxDis = Path.getLength(); // In Inches

// //       this->dt = dt;  // In Seconds

// //       this->dd = dd;  // In Inches

// //       Velocities = generate();
// //     }

// //     vector<double> generate() {
// //       double t = 0;

// //       double dist = 0;

// //       double Velo = 0;

// //       double lastAngularVel = 0;

// //       maxDis = Path.getLength();

// //       vector<double> Velos;

// //       while (dist < maxDis && t < Path.getSegments()) {
// //         Velo = getVelocty(dist, Velo);

// //         double curvature = Path.getCurvature(t);

// //         double AngVel = Velo * curvature; // Not Sure why this is here
// //         double AngAccel = (AngVel - lastAngularVel) * (Velo / dd);
// //         lastAngularVel = AngVel;

// //         double mA = MAX_ACCELERATION - abs(AngAccel * TRACKWIDTH / 2);

// //         Velo = min(GetMaxSpeed(curvature), sqrt(Velo * Velo + 2 * mA * dd));

// //         Velos.push_back(Velo);

// //         dist += Velo * dt;

// //         t = Path.get_t_at_dist(dist);
// //       }
// //       return Velos;
// //     }

// //     vector<double> getMotionProfile() {
// //       return Velocities;
// //     }

// //   private:
// //   vector<double> Velocities;

// //   CatmullRom Path;

// //   // Kinematics K;

// //   double accelDis = 25;
// //   double maxDis;
// //   double dt = 1, dd = 1;

// //   double getVelocty(double d, double initialVelo) {
// //     if (d < accelDis) {
// //       return sqrt(initialVelo * initialVelo + 2 * MAX_ACCELERATION * d);
// //     } else if (d > accelDis && d < (maxDis - accelDis)) {
// //       return MAX_VELOCITY;
// //     } else {
// //       return sqrt(initialVelo * initialVelo + 2 * MAX_ACCELERATION * (d - maxDis));
// //     }
// //   }
  
// // };


























































// // class Trajectory {
// //   public:
// //   /* Constructors */
// //     /** 
// //     * @brief Creates an instance of a Trajectory Rom Object
// //     */
// //     Trajectory() {}

// //     // Default Empty Destructor
// //     ~Trajectory() {}

// //     // Default Non-Empty Constructor
// //     Trajectory(std::vector<Point> controlPoints, double accelDis, double dt = 1, double dd = 1) {
// //       this-> Spline = CatmullRom(controlPoints);
      
// //       this->accelDis = accelDis; // In Inches

// //       this->maxDis = Spline.getDist(); // In Inches

// //       this->dt = dt;  // In Seconds

// //       this->dd = 1;  // In Inches

// //       // Make a Robot Constraints and Model
// //       double TrackWidth = 15; // In Inches
// //       double maxVel = 5; // In Inches Per Sec
// //       double maxAccel = 10; // In Inches Per Sec ^ 2

// //       Path.clear();
// //       // Linear Velo
// //       // vector<Point> temp;
// //       double dist = 1, v = 0;
// //       while (dist <= maxDis) {
// //         // Setting the Position (Has Curvature Alr) of the Point
// //         Point profilePoint = Spline.interpolate(Spline.get_t_at_dist(dist));
// //         profilePoint.PrintPoint();

// //         /* Finding the Linear Velocity */
// //         v = getVelocty(dist, v);
// //         cout << v << endl;

// //         profilePoint.SetVelocity(min(v, maxSpeed(profilePoint.GetCurve())));

// //         // Increasing Distance
// //         dist += v * dt;

// //         // Adding the Point
// //         Path.push_back(profilePoint);
// //       }
// //       // Path = temp;
// //     }

// //   /* Functions */

// //     void generatePath(void) {
// //       Path.clear();
// //       // Linear Velo
// //       double dist, v = 0;
// //       while (dist <= maxDis) {
// //         // Setting the Position (Has Curvature Alr) of the Point
// //         Point profilePoint = Spline.interpolate(Spline.get_t_at_dist(dist));

// //         /* Finding the Linear Velocity */
// //         v = getVelocty(dist, v);

// //         profilePoint.SetVelocity(min(v, maxSpeed(profilePoint.GetCurve())));

// //         // Increasing Distance
// //         dist += v * dt;

// //         // Adding the Point
// //         Path.push_back(profilePoint);
// //       }
// //     }

// //     double getVelocty(double d, double initialVelo) {
// //       if (d < accelDis) {
// //         return sqrt(initialVelo * initialVelo + 2 * maxAccel * d);
// //       } else if (d > accelDis && d < (maxDis - accelDis)) {
// //         return maxVel;
// //       } else {
// //         return sqrt(initialVelo * initialVelo + 2 * maxAccel * (d - maxDis));
// //       }
// //     }

// //     double size(void) {
// //       return Path.size();
// //     }

// //     Point operator[](double index) {
// //       return Path[index];
// //     }

// //   private:
// //   /* Instance Data */
// //   double accelDis = 1;
// //   double maxDis;
// //   double TrackWidth = 15;
// //   // dt = 50 Miliseconds, dd = 1 inch
// //   double dt = 1;
// //   double dd = 1;

// //   // Positional Data
// //   CatmullRom Spline;


// //   // Final Trajectory
// //   vector<Point> Path;

// //   double maxVel;
// //   double maxAccel;

// //   double maxSpeed(double curvature) {
// //     return ((2 * maxVel / TrackWidth) * maxVel) / (fabs(curvature) * maxVel + (2 * maxVel / TrackWidth));
// //   }
// // };






// /* Abeeto Implementation */
// // class Trajectory {
// //   public:
// //   /* Constructors */
// //     /** 
// //     * @brief Creates an instance of a Trajectory Rom Object
// //     */
// //     Trajectory() {}

// //     // Default Empty Destructor
// //     ~Trajectory() {}

// //     struct ProfilePoint {
// //       Point p;
// //       double vel;
// //       double angularVel;
// //       double dist;
// //       double t;
// //     };

// //   void generate() {
// //     struct IntermediateProfilePoint {
// //       double vel;
// //       double dist;
// //       double t;
// //       double curvature;
// //     };

// //     int numPoints = Path.getLength() / deltaDistance;

// //     std::vector<IntermediateProfilePoint> forwardPass;
// //     forwardPass.reserve(numPoints);

// //     double vel = 0;
// //     double lastAngularVel = 0;

// //     forwardPass.push_back({0, 0, 0, Path.getCurvature(0)});

// //     for (int i = 1; numPoints > 2 && i < numPoints; i++) {
// //         double d = deltaDistance * i;
// //         double t = Path.get_t_at_dist(d);

// //         double curvature = Path.getCurvature(t);
// //         double angularVel = vel * curvature;
// //         double angularAccel = (angularVel - lastAngularVel) * (vel / deltaDistance);
// //         lastAngularVel = angularVel;

// //         double maxAccel = K.MaxAccel - std::abs(angularAccel * K.TrackWidth / 2);
// //         vel = std::min(K.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * deltaDistance));

// //         forwardPass.push_back(IntermediateProfilePoint {vel, d, t, curvature});
// //     }

// //     forwardPass.push_back({0, Path.getLength(), Path.getSegments(), Path.getCurvature(Path.getSegments())});

// //     Profile.clear();
// //     Profile.reserve(numPoints);

// //     vel = 0;
// //     lastAngularVel = 0;

// //     for (int i = numPoints - 1; i >= 0; i--) {
// //         IntermediateProfilePoint correspondingProfilePoint = forwardPass[i];

// //         double angularVel = vel * correspondingProfilePoint.curvature;
// //         double angularAccel = (angularVel - lastAngularVel) * (vel / deltaDistance);
// //         lastAngularVel = angularVel;

// //         double maxAccel = K.MaxDecel - std::abs(angularAccel * K.TrackWidth / 2);
// //         vel = std::min(K.GetMaxSpeed(correspondingProfilePoint.curvature), std::sqrt(vel * vel + 2 * maxAccel * deltaDistance));

// //         double minVel = std::min(vel, correspondingProfilePoint.vel);

// //         Profile.push_back({
// //             Path.interpolate(correspondingProfilePoint.t),
// //             minVel,
// //             minVel * correspondingProfilePoint.curvature,
// //             correspondingProfilePoint.dist,
// //             NAN
// //         });
// //     }

// //     // std::reverse(m_ProfilePoints.begin(), m_ProfilePoints.end());

// //     double t = 0;

// //     for (ProfilePoint& point : Profile) {
// //       t += (point.vel == 0) ? 0 : deltaDistance / point.vel;
// //       point.t = t;
// //     }
// // }


  

// //   private:
// //   std::vector<ProfilePoint> Profile;
// //   CatmullRom Path;
// //   Kinematics K;

// //   double deltaDistance = 1;


  
// // };