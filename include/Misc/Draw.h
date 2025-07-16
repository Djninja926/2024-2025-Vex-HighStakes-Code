#pragma once

#include <string>
#include <vector>
#include <cstring>
#include <sstream>
#include <iostream>

namespace Draw {
  // Brain Screen UI
  extern int ScreenMode;           
  extern int SubScreen; // Sub Screens For each thing
  extern int Set0;
  extern int Controller;
  extern int XPOSI, YPOSI, Next;
  // Field Drawing
  extern char *Algo;
  extern char *Motors;
  extern double Scale;
  extern double PredictX, PredictY;
  extern int BorderWidth;
  extern int Side; // 0 = Left, 1 = Right
  extern int Tiles, TileWidth;
  extern int StartPositionX, StartPositionY; // 252 with Scale of 1.5 and border width of 12
  extern int BarrierWidth;
  // Pure Pursuit Graph Vars
  extern std::vector<double> LeftVelo;
  extern std::vector<double> LeftActualVelo;
  extern std::vector<double> RightVelo;
  extern std::vector<double> RightActualVelo;
  // Functions
  extern void Cycle(void);
  extern double LimitPower(double POWER);
  extern double LimitDis(void);
  extern void DrawAutoRoutes(int Auton);
  // extern void DrawBoomerang(int Start);
  // extern void DrawPursuitRoute(int Start);
  extern void DrawField(void);
}