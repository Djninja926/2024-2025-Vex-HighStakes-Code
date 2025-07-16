// Default
#include "vex.h"
#include "cmath"
#include "api.h"

// Necessary Libraries
#include <string>
#include <cstring>
#include <sstream>

namespace Draw {
  // Brain Screen UI
  int ScreenMode = 1;           
  int SubScreen = 0; // Sub Screens For each thing
  int Set0 = 1;
  int Controller = 1;
  int XPOSI = 0, YPOSI = 0, Next = 0;
  // Field Drawing
  char *Algo = (char*) "Algo:: PurePursuit";
  char *Motors = (char*) "L1";
  double Scale = 1.5;
  double PredictX, PredictY;
  int BorderWidth = 12;
  int Side = 0; // 0 = Left, 1 = Right
  int Tiles = 24;  
  int TileWidth = Scale * Tiles;
  int StartPositionX = 480 - TileWidth * 6 - BorderWidth; // 252 with Scale of 1.5 and border width of 12
  int StartPositionY = BorderWidth;
  int BarrierWidth = TileWidth / 6;
  // Pure Pursuit Graph Vars
  std::vector<double> LeftVelo;
  std::vector<double> LeftActualVelo;
  std::vector<double> RightVelo;
  std::vector<double> RightActualVelo;
  // Functions
  void Cycle(void);
  double LimitPower(double POWER);
  double LimitDis(void);
  void DrawAutoRoutes(int Auton);
  // void DrawBoomerang(int Start);
  // void DrawPursuitRoute(int Start);
  void DrawField(void);
}

void Draw::DrawField(void) {
  while (true) {
    Brain.Screen.waitForRefresh();
    Brain.Screen.render(true);
    Brain.Screen.clearScreen("#262728");

    if (Set0 != 0) {
      Set0 = 0;
      XPOSI = 0;
      YPOSI = 0;
    } else if (Brain.Screen.pressing()) {
      XPOSI = Brain.Screen.xPosition();
      YPOSI = Brain.Screen.yPosition();
    }
    if (ScreenMode == 1) {
      // Box Around
      Brain.Screen.setPenWidth(3);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor(black);
      Brain.Screen.drawRectangle(StartPositionX - 4, StartPositionY - 4, (TileWidth * 6) + 6, (TileWidth * 6) + 6);
      // Tiles
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor(black);
      for (int Row = 0; Row < 6; Row++) {
        for (int Column = 0; Column < 6; Column++) {
          Brain.Screen.drawRectangle(StartPositionX + TileWidth * Row, StartPositionY + TileWidth * Column, TileWidth, TileWidth, "#939293");
        }
      }
      
      // Middle Tower
      Brain.Screen.setPenColor(yellow);
      Brain.Screen.setPenWidth(4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2, StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4, StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3);

      // White Lines
      Brain.Screen.setPenColor(white);
      Brain.Screen.setPenWidth(2);
      Brain.Screen.drawLine(StartPositionX + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth * 5 + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth * 5 + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);

      // Positve Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);

      // Negative Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY  + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);

      // Draw Robot
      Brain.Screen.setPenWidth(1);
      if (!Side) {
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawCircle(MCL::X * Scale + StartPositionX + 3 * TileWidth, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, 18, transparent);
        // Draw Angle
        Brain.Screen.setPenColor(174);
        PredictX = (MCL::X * Scale + StartPositionX + 3 * TileWidth) + 20 * sin(MCL::theta);
        PredictY = (-MCL::Y * Scale + StartPositionY + 3 * TileWidth) + 20 * -cos(MCL::theta);
        Brain.Screen.drawLine(MCL::X * Scale + StartPositionX + 3 * TileWidth, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
      } else if (Side) {
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawCircle(MCL::X * Scale + StartPositionX + TileWidth * 5 - 11, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, 18, transparent);
        // Draw Angle
        Brain.Screen.setPenColor(174);
        PredictX = (MCL::X * Scale + StartPositionX + TileWidth * 5 - 11) + 20 * sin(MCL::theta);
        PredictY = (-MCL::Y * Scale + StartPositionY + 3 * TileWidth) + 20 * -cos(MCL::theta);
        Brain.Screen.drawLine(MCL::X * Scale + StartPositionX + TileWidth * 5 - 11, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
      }

      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.drawRectangle(10, 10, 228, 40);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFont(monoL);
      Brain.Screen.printAt(16, 10 + 28, "1082R: R.G.V.V");
      Brain.Screen.setFont(monoM);

      // Buttons
      // Auton Button
      Brain.Screen.setPenColor(blue); // 174 - light blue
      Brain.Screen.setPenWidth(1);
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 120) && (YPOSI > 60 && YPOSI < 130)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(blue); // 174 - light blue 
        Brain.Screen.drawRectangle(10, 60, 110, 70);
        Brain.Screen.printAt(23, 90, "Auton");
        Brain.Screen.printAt(23, 110, "Selector");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 120) && (YPOSI > 60 && YPOSI < 130) && ScreenMode != 2) {
        ScreenMode = 2;
        Set0 = 1;
        Next = 0;
      } else {
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(10, 60, 110, 70);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.printAt(23, 90, "Auton");
        Brain.Screen.printAt(23, 110, "Selector");
      }
      // Color Buton
      Brain.Screen.setPenColor(blue); // 174 - light blue
      Brain.Screen.setPenWidth(1);
      if (Brain.Screen.pressing() && (XPOSI > 140 && XPOSI < 260) && (YPOSI > 60 && YPOSI < 130)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(blue); // 174 - light blue 
        Brain.Screen.drawRectangle(130, 60, 110, 70);
        Brain.Screen.printAt(140, 90, "Localize");
        Brain.Screen.printAt(140, 110, "Directory");
      } else if (!Brain.Screen.pressing() && (XPOSI > 140 && XPOSI < 260) && (YPOSI > 60 && YPOSI < 130) && ScreenMode != 3) {
        ScreenMode = 3;
        Set0 = 1;
      } else {
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(130, 60, 110, 70);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.printAt(140, 90, "Localize");
        Brain.Screen.printAt(140, 110, "Directory");
      }
      // Roller Button
      Brain.Screen.setPenColor(blue); // 174 - light blue
      Brain.Screen.setPenWidth(1);
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 130) && (YPOSI > 150 && YPOSI < 230)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(blue); // 174 - light blue 
        Brain.Screen.drawRectangle(10, 150, 110, 70);
        Brain.Screen.printAt(20, 180, "Robot");
        Brain.Screen.printAt(20, 200, "Central");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 130) && (YPOSI > 150 && YPOSI < 230) && ScreenMode != 4) {
        ScreenMode = 4;
        Set0 = 1;
      } else {
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(10, 150, 110, 70);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.printAt(20, 180, "Robot");
        Brain.Screen.printAt(20, 200, "Central");
      }
      // MCL Stuff
      Brain.Screen.setPenColor(blue); // 174 - light blue
      Brain.Screen.setPenWidth(1);
      if (Brain.Screen.pressing() && (XPOSI > 140 && XPOSI < 260) && (YPOSI > 150 && YPOSI < 230)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(blue); // 174 - light blue
        Brain.Screen.drawRectangle(130, 150, 110, 70);
        Brain.Screen.printAt(140, 180, "Motion");
        Brain.Screen.printAt(140, 200, "Algo's");
      } else if (!Brain.Screen.pressing() && (XPOSI > 140 && XPOSI < 260) && (YPOSI > 150 && YPOSI < 230) && ScreenMode != 5) {
        ScreenMode = 5;
        Set0 = 1;
        Controller = 1;
      } else {
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(130, 150, 110, 70);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.printAt(140, 180, "Motion");
        Brain.Screen.printAt(140, 200, "Algo's");
      }
    } else if (ScreenMode == 2) {
      // Box Around
      Brain.Screen.setPenWidth(3);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor(black);
      Brain.Screen.drawRectangle(StartPositionX - 4, StartPositionY - 4, (TileWidth * 6) + 6, (TileWidth * 6) + 6);
      // Tiles
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor(black);
      for (int Row = 0; Row < 6; Row++) {
        for (int Column = 0; Column < 6; Column++) {
          Brain.Screen.drawRectangle(StartPositionX + TileWidth * Row, StartPositionY + TileWidth * Column, TileWidth, TileWidth, "#939293");
        }
      }
      
      // Middle Tower
      Brain.Screen.setPenColor(yellow);
      Brain.Screen.setPenWidth(4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2, StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4, StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3);

      // White Lines
      Brain.Screen.setPenColor(white);
      Brain.Screen.setPenWidth(2);
      Brain.Screen.drawLine(StartPositionX + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth * 5 + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth * 5 + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);

      // Positve Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);

      // Negative Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY  + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);

      if (SubScreen == 0) {
        for (int x = 30, ButtonRow = 1; x <= 130; x += 100, ButtonRow += 4) {
          // Brain.Screen.setPenColor(white);
          Brain.Screen.setFillColor("#262728");
          if (!Next) {
            Brain.Screen.setPenColor(red);
            Brain.Screen.printAt(52, 30, "Red");
            Brain.Screen.setPenColor(blue);
            Brain.Screen.printAt(152, 30, "Blue");
          } else {
            Brain.Screen.setPenColor(red);
            Brain.Screen.printAt(52, 30, "Red");
            Brain.Screen.setPenColor(blue);
            Brain.Screen.printAt(152, 30, "Blue");
          }

          Brain.Screen.setPenColor(white);

          for (int z = 40, ButtonColumn = 0; z <= 205; z += 50, ButtonColumn++) {
            // Setting the Auto Name
            if (Next == 0) {
              if (ButtonRow + ButtonColumn == 1) {
                Auto = (char*) "Rings"; // Useless
              } else if (ButtonRow + ButtonColumn == 2) {
                Auto = (char*) "F-AWP";
              } else if (ButtonRow + ButtonColumn == 3) {
                Auto = (char*) "Safe"; // Useless
              } else if (ButtonRow + ButtonColumn == 4) {
                Auto = (char*) "G-Rush";
              } else if (ButtonRow + ButtonColumn == 5) {
                Auto = (char*) "Rings";
              } else if (ButtonRow + ButtonColumn == 6) {
                Auto = (char*) "F-AWP";
              } else if (ButtonRow + ButtonColumn == 7) {
                Auto = (char*) "Safe";
              } else if (ButtonRow + ButtonColumn == 8) {
                Auto = (char*) "G-Rush";
              }
            } else if (Next == 1) {
              if (ButtonRow + ButtonColumn == 1) {
                Auto = (char*) "Safe"; // Useless
              } else if (ButtonRow + ButtonColumn == 2) {
                Auto = (char*) "Risky";
              } else if (ButtonRow + ButtonColumn == 3) {
                Auto = (char*) "R-Rush"; // Useless
              } else if (ButtonRow + ButtonColumn == 4) {
                Auto = (char*) "FAWP";
              } else if (ButtonRow + ButtonColumn == 5) {
                Auto = (char*) "Safe";
              } else if (ButtonRow + ButtonColumn == 6) {
                Auto = (char*) "G-Rush";
              } else if (ButtonRow + ButtonColumn == 7) {
                Auto = (char*) "NA";
              } else if (ButtonRow + ButtonColumn == 8) {
                Auto = (char*) "Next";
              }
            } else if (Next == 2) {
              if (ButtonRow + ButtonColumn == 1) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 2) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 3) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 4) {
                Auto = (char*) "Back";
              } else if (ButtonRow + ButtonColumn == 5) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 6) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 7) {
                Auto = (char*) "";
              } else if (ButtonRow + ButtonColumn == 8) {
                Auto = (char*) "";
              }
            }

            // Auton Buttons
            if (Brain.Screen.pressing() && (XPOSI > x && XPOSI < x + 90) && (YPOSI > z && YPOSI < z + 45)) {
              Brain.Screen.setFillColor(blue);
              Brain.Screen.setPenColor(black);
              Brain.Screen.setPenWidth(3);
              Brain.Screen.drawRectangle(x, z, 90, 45);
              Brain.Screen.setPenColor(black);
              Brain.Screen.setPenWidth(1);
              Brain.Screen.printAt(x + 14, z + 28, Auto);
              DrawAutoRoutes(ButtonRow + ButtonColumn);
            } else if (!Brain.Screen.pressing() && (XPOSI > x && XPOSI < x + 90) && (YPOSI > z && YPOSI < z + 45) && SubScreen != 1) {
              // DrawAutoRoutes(ButtonRow + ButtonColumn);
              if (Next == 0) {
                // if (ButtonRow + ButtonColumn == 8) {
                  // Next = 1;
                // } else {
                  autonType = ButtonRow + ButtonColumn;
                  // thread Calc(Auton::CalcFirstPath);
                  SubScreen = 1;
                  Controller = 1;
                // }
              } else if (Next == 1) {
                if (ButtonRow + ButtonColumn == 8 /* || ButtonRow + ButtonColumn == 4 */) {
                  // if (ButtonRow + ButtonColumn == 4) {
                    // Next = 0;
                  // } else {
                    Next = 2;
                  // }
                } else {
                  autonType = ButtonRow + ButtonColumn + 7;
                  // thread Calc(Auton::CalcFirstPath);
                  SubScreen = 1;
                  Controller = 1;
                }
              } else if (Next == 2) {
                if (ButtonRow + ButtonColumn == 4) {
                  Next = 1;
                } else {
                  autonType = ButtonRow + ButtonColumn + 14;
                  // thread Calc(Auton::CalcFirstPath);
                  SubScreen = 1;
                  Controller = 1;
                }
              }
              Set0 = 1;
            } else {
              Brain.Screen.setPenWidth(1);
              Brain.Screen.setPenColor(black);
              Brain.Screen.setFillColor("#121314");
              Brain.Screen.drawRectangle(x, z, 90, 45);
              Brain.Screen.setPenColor(white);
              Brain.Screen.printAt(x + 14, z + 28, Auto);
            }
          }
        }
      } else if (SubScreen == 1) {
        Auto = (char*)" ";
        // Drawing the Auton Route
        DrawAutoRoutes(autonType);
        // Choosing the Auto
        if (autonType == 1) {
          Auto = (char*) "Red Ring Side Rush";
          Side = 0;
        } else if (autonType == 2) {
          Auto = (char*) "Red Ring Solo AWP";
          Side = 0;
        } else if (autonType == 3) {
          Auto = (char*) "Red Goal Side Safe";
          Side = 0;
        } else if (autonType == 4) {
          Auto = (char*) "Red Goal Rush";
          Side = 0;
        } else if (autonType == 5) {
          Auto = (char*) "Blue Ring Side Rush";
          Side = 1;
        } else if (autonType == 6) {
          Auto = (char*) "Blue Ring Solo AWP";
          Side = 1;
        } else if (autonType == 7) {
          Auto = (char*) "Blue Goal Side Safe";
          Side = 1;
        } else if (autonType == 8) {
          Auto = (char*) "Blue Goal Rush";
          Side = 0;
        } else if (autonType == 9) {
          Auto = (char*) "RR, 3 ???";
          Side = 0;
        } else if (autonType == 10) {
          Auto = (char*) "";
          Side = 0;
        } else if (autonType == 12) {
          Auto = (char*) "BR, 5 ???";
          Side = 0;
        } else if (autonType == 13) {
          Auto = (char*) "BL, 3 ???";
          Side = 0;
        } else if (autonType == 14) {
          Auto = (char*) "";
          Side = 0;
        }
        // Printing to the Controller
        if (Controller) {
          Controller1.Screen.clearLine(3);
          Controller1.Screen.print(Auto);
          Controller = 0;
        }
        // The Auto Box
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#262728");
        Brain.Screen.printAt(6, 165, "Auto Selected");
        Brain.Screen.setPenWidth(2);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(6, 175, 235, 50);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.printAt(12, 207, Auto);

        // Yes and No Buttons
        if (Brain.Screen.pressing() && (XPOSI > 6 && XPOSI < 241) && (YPOSI > 10 && YPOSI < 70)) {
          // Green Box
          Brain.Screen.setPenWidth(3);
          Brain.Screen.setFillColor(green);
          Brain.Screen.setPenColor(black);
          Brain.Screen.drawRectangle(6, 10, 235, 60);
          // Red Box
          Brain.Screen.setPenWidth(1);
          Brain.Screen.setFillColor("#121314");
          Brain.Screen.setPenColor(blue);
          Brain.Screen.drawRectangle(6, 80, 235, 60);

          Brain.Screen.setFillColor(green);
          Brain.Screen.setPenColor(black);
          Brain.Screen.printAt(38, 47, "Confirm Selection");
          Brain.Screen.setFillColor("#121314");
          Brain.Screen.setPenColor(white);
          Brain.Screen.printAt(55, 117, "Redo Selection");
        } else if (Brain.Screen.pressing() && (XPOSI > 6 && XPOSI < 241) && (YPOSI > 80 && YPOSI < 150)) {
          // Red Box
          Brain.Screen.setPenWidth(3);
          Brain.Screen.setFillColor(red);
          Brain.Screen.setPenColor(black);
          Brain.Screen.drawRectangle(6, 80, 235, 60);
          // Green Box
          Brain.Screen.setPenWidth(1);
          Brain.Screen.setFillColor("#121314");
          Brain.Screen.setPenColor(blue);
          Brain.Screen.drawRectangle(6, 10, 235, 60);

          Brain.Screen.setFillColor("#121314");
          Brain.Screen.setPenColor(white);
          Brain.Screen.printAt(38, 47, "Confirm Selection");
          Brain.Screen.setFillColor(red);
          Brain.Screen.setPenColor(black);
          Brain.Screen.printAt(55, 117, "Redo Selection");
        } else if (!Brain.Screen.pressing() && (XPOSI > 6 && XPOSI < 241) && (YPOSI > 10 && YPOSI < 70) && ScreenMode != 1) {
          ScreenMode = 1;
          SubScreen = 0;
          Set0 = 1;
        } else if (!Brain.Screen.pressing() && (XPOSI > 6 && XPOSI < 241) && (YPOSI > 80 && YPOSI < 150) && ScreenMode != 1) {
          ScreenMode = 2;
          SubScreen = 0;
          Set0 = 1;
        } else {
          Brain.Screen.setPenWidth(1);
          Brain.Screen.setPenColor(blue);
          Brain.Screen.setFillColor("#121314");
          Brain.Screen.drawRectangle(6, 10, 235, 60);
          Brain.Screen.drawRectangle(6, 80, 235, 60);
          Brain.Screen.setPenColor(white);
          Brain.Screen.printAt(38, 47, "Confirm Selection");
          Brain.Screen.printAt(55, 117, "Redo Selection");
        }
      }
    } else if (ScreenMode == 3) {
      // Box for X, Y, Angle
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.setPenColor(white);

      Brain.Screen.drawRectangle(5, 130, (TileWidth * 4) - 48, (TileWidth * 4) - 44);

      Brain.Screen.drawRectangle(5 + (TileWidth * 4) - 24, 130, (TileWidth * 4) - 48, (TileWidth * 4) - 44);

      Brain.Screen.drawRectangle(5 + (TileWidth * 4) - 24, 130 - (TileWidth * 4) + 24, (TileWidth * 4) - 48, (TileWidth * 4) - 40);

      // X, Y, Angle
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFillColor("#121314");

      // Brain.Screen.printAt(17, 155, "X: %.2f", MCL::X);
      // Brain.Screen.printAt(17, 185, "Y: %.2f", MCL::Y);
      // Brain.Screen.printAt(17, 215, "θ: %.2f", ToDegrees(MCL::AngleWrap(MCL::Angle)));

      Brain.Screen.printAt(17, 155, "X: %.2f", MCL::X);
      Brain.Screen.printAt(17, 185, "Y: %.2f", MCL::Y);
      Brain.Screen.printAt(17, 215, "θ: %.2f", ToDegrees(AngleWrap(MCL::theta)));

      // Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 155, "X: %.2f", State::getState()(4, 0));
      // Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 185, "Y: %.2f", State::getState()(2, 0));
      // Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 215, "θ: %.2f", State::getState()(2, 0));
      // Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 215, "θ: %.2f", ToDegrees((State::getState()(2, 0))));

      Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 155 - (TileWidth * 4) + 24, "X: %.2f", MCL::X);
      Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 185 - (TileWidth * 4) + 24, "Y: %.2f", MCL::Y);
      Brain.Screen.printAt(17 + (TileWidth * 4) - 24, 215 - (TileWidth * 4) + 24, "θ: %.2f", ToDegrees(AngleWrap(MCL::theta)));

      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor("#262728");
      for (int Row = 0; Row < 12; Row++) {
        for (int Column = 0; Column < 12; Column++) {
          Brain.Screen.drawRectangle(StartPositionX + TileWidth/2 * Row, StartPositionY + TileWidth/2 * Column, TileWidth/2, TileWidth/2, "transparent");
        }
      }

      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(white);
      for (int Row = 0; Row < 2; Row++) {
        for (int Column = 0; Column < 2; Column++) {
          Brain.Screen.drawRectangle(StartPositionX + TileWidth * 3 * Row, StartPositionY + TileWidth * 3 * Column, TileWidth * 3, TileWidth * 3, transparent);
        }
      }

      // Draw Robot
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor(174);
      Brain.Screen.drawCircle(MCL::X * Scale / 3 + (StartPositionX + TileWidth * 3), -MCL::Y * Scale / 3 + (StartPositionY + TileWidth * 3), 18, transparent);
      
      // Draw Angle
      Brain.Screen.setPenColor(red); // "#121314"
      PredictX = (MCL::X * Scale / 3 + (StartPositionX + TileWidth * 3)) + 20 * sin(MCL::theta);
      PredictY = (-MCL::Y * Scale / 3 + (StartPositionY + TileWidth * 3)) - 20 * cos(MCL::theta);
      Brain.Screen.drawLine(MCL::X * Scale / 3 + (StartPositionX + TileWidth * 3), -MCL::Y * Scale / 3 + (StartPositionY + TileWidth * 3), PredictX, PredictY);

      // Draw Buttons
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 70 && YPOSI < 115)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(white);
        Brain.Screen.drawRectangle(10, 70, 90, 45);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(black);
        Brain.Screen.printAt(10 + 21, 70 + 28, "Reset");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 70 && YPOSI < 115)) {
        // MCL::ResetMCL();
        Set0 = 1;
      } else {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(10, 70, 90, 45);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10 + 21, 70 + 28, "Reset");
      }
      // Home Button
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 10 && YPOSI < 55)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(white);
        Brain.Screen.drawRectangle(10, 10, 90, 45);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(black);
        Brain.Screen.printAt(10 + 23, 10 + 28, "Home");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 10 && YPOSI < 55) && ScreenMode != 1) {
        ScreenMode = 1;
        Set0 = 1;
      } else {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(10, 10, 90, 45);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10 + 23, 10 + 28, "Home");
      }
    } else if (ScreenMode == 4) {
      // Gonna Be Robot Health Stuff
      // Home Button
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 10 && YPOSI < 55)) {
        Brain.Screen.setPenWidth(3);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor(white);
        Brain.Screen.drawRectangle(10, 10, 90, 45);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(black);
        Brain.Screen.printAt(10 + 23, 10 + 28, "Home");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 100) && (YPOSI > 10 && YPOSI < 55) && ScreenMode != 1) {
        ScreenMode = 1;
        Set0 = 1;
      } else {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(10, 10, 90, 45);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10 + 23, 10 + 28, "Home");
      }

      for (int x = 105, ButtonRow = 1; x <= 255; x += 150, ButtonRow += 4) {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#262728");
        Brain.Screen.printAt(152 + 85, 30, "Left");
        Brain.Screen.printAt(292 + 95, 30, "Right");
        for (int z = 40, ButtonColumn = 0; z <= 250; z += 50, ButtonColumn++) {
          motor temp = motor(PORT21);
          if (ButtonRow + ButtonColumn == 1) {
            temp = L1;
            Motors = (char*) "L1: ";
          } else if (ButtonRow + ButtonColumn == 2) {
            temp = L2;
            Motors = (char*) "L2: ";
          } else if (ButtonRow + ButtonColumn == 3) {
            temp = L3;
            Motors = (char*) "L3: ";
          } else if (ButtonRow + ButtonColumn == 4) {
            temp = Lift;
            Motors = (char*) "Lift: ";
          } else if (ButtonRow + ButtonColumn == 5) {
            temp = R1;
            Motors = (char*) "R1: ";
          } else if (ButtonRow + ButtonColumn == 6) {
            temp = R2;
            Motors = (char*) "R2: ";
          } else if (ButtonRow + ButtonColumn == 7) {
            temp = R3;
            Motors = (char*) "R3: ";
          } else if (ButtonRow + ButtonColumn == 8) {
            temp = Intake;
            Motors = (char*) "In: ";
          }
          Brain.Screen.setPenWidth(1);
          Brain.Screen.setPenColor(black);
          Brain.Screen.setFillColor("#121314");
          Brain.Screen.drawRectangle(x + 100, z, 120, 45);
          Brain.Screen.setPenColor(white);
          Brain.Screen.printAt(x + 107, z + 28, true, "%2s%.2f%s", Motors, temp.temperature(fahrenheit), "°");
        }
      }
    } else if (ScreenMode == 5) {
      // Box Around
      Brain.Screen.setPenWidth(3);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor(black);
      Brain.Screen.drawRectangle(StartPositionX - 4, StartPositionY - 4, (TileWidth * 6) + 6, (TileWidth * 6) + 6);
      // Tiles
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor(black);
      for (int Row = 0; Row < 6; Row++) {
        for (int Column = 0; Column < 6; Column++) {
          Brain.Screen.drawRectangle(StartPositionX + TileWidth * Row, StartPositionY + TileWidth * Column, TileWidth, TileWidth, "#939293");
        }
      }
      
      // Middle Tower
      Brain.Screen.setPenColor(yellow);
      Brain.Screen.setPenWidth(4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 2, StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 4, StartPositionY + TileWidth * 3, StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 3, StartPositionY + TileWidth * 4, StartPositionX + TileWidth * 2, StartPositionY + TileWidth * 3);

      // White Lines
      Brain.Screen.setPenColor(white);
      Brain.Screen.setPenWidth(2);
      Brain.Screen.drawLine(StartPositionX + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + 1, StartPositionY + TileWidth * 5 + TileWidth / 2, StartPositionX + TileWidth * 6 - 2, StartPositionY + TileWidth * 5 + TileWidth / 2);
      Brain.Screen.drawLine(StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + 2, StartPositionX + TileWidth * 5 + TileWidth / 2, StartPositionY + TileWidth * 6 - 2);

      // Positve Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, green);

      // Negative Corners
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawRectangle(StartPositionX + 2, StartPositionY  + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);
      Brain.Screen.drawRectangle(StartPositionX + TileWidth * 5 + TileWidth / 2 + 2, StartPositionY + TileWidth * 5 + TileWidth / 2 + 2, TileWidth / 2 - 4, TileWidth / 2 - 4, red);
      
      // Draw Robot
      if (!Side) {
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawCircle(MCL::X * Scale + StartPositionX + 3 * TileWidth, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, 18, transparent);
        // Draw Angle
        Brain.Screen.setPenColor(174);
        PredictX = (MCL::X * Scale + StartPositionX + 3 * TileWidth) + 20 * sin(MCL::theta);
        PredictY = (-MCL::Y * Scale + StartPositionY + 3 * TileWidth) + 20 * -cos(MCL::theta);
        Brain.Screen.drawLine(MCL::X * Scale + StartPositionX + 3 * TileWidth, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
      } else if (Side) {
        Brain.Screen.setPenColor(green);
        Brain.Screen.drawCircle(MCL::X * Scale + StartPositionX + TileWidth * 5 - 11, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, 18, transparent);
        // Draw Angle
        Brain.Screen.setPenColor(174);
        PredictX = (MCL::X * Scale + StartPositionX + TileWidth * 5 - 11) + 20 * sin(MCL::theta);
        PredictY = (-MCL::Y * Scale + StartPositionY + 3 * TileWidth) + 20 * -cos(MCL::theta);
        Brain.Screen.drawLine(MCL::X * Scale + StartPositionX + TileWidth * 5 - 11, -MCL::Y * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
      }

      // Draw Graph Back Ground
      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.drawRectangle(6, 100, 235, 125);
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor("#262728");
      Brain.Screen.setFillColor("#121314");

      // Drawing the graph lines
      for (int Graph = 104; Graph < 230; Graph += 10) {
        Brain.Screen.drawLine(6, Graph, 241, Graph);
      }

      // Home Button
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 120) && (YPOSI > 10 && YPOSI < 55)) {
        Brain.Screen.setPenWidth(2);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(6, 10, 112, 45);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10 + 34, 10 + 28, "Home");
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 120) && (YPOSI > 10 && YPOSI < 55) && ScreenMode != 1) {
        ScreenMode = 1;
        SubScreen = 0;
        Set0 = 1;
      } else {
        Brain.Screen.setPenWidth(2);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(6, 10, 112, 45);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10 + 34, 10 + 28, "Home");
      }

      // Next/Cycle Button
      if (Brain.Screen.pressing() && (XPOSI > 130 && XPOSI < 240) && (YPOSI > 10 && YPOSI < 55)) {
        Brain.Screen.setPenWidth(2);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(128, 10, 112, 45);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(130 + 10, 10 + 28, "Next Algo");
      } else if (!Brain.Screen.pressing() && (XPOSI > 130 && XPOSI < 240) && (YPOSI > 10 && YPOSI < 55) && ScreenMode != 1) {
        Controller = 1;
        Set0 = 1;
        Cycle();
      } else {
        Brain.Screen.setPenWidth(2);
        Brain.Screen.setPenColor(black);
        Brain.Screen.setFillColor("#121314");
        Brain.Screen.drawRectangle(128, 10, 112, 45);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(130 + 10, 10 + 28, "Next Algo");
      }

      // Graph Enlarge Button Button
      if (Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 245) && (YPOSI > 100 && YPOSI < 225)) {
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor(transparent);
        Brain.Screen.drawRectangle(6, 100, 236, 126);
      } else if (!Brain.Screen.pressing() && (XPOSI > 10 && XPOSI < 245) && (YPOSI > 100 && YPOSI < 225) && ScreenMode != 1) {
        ScreenMode = 6;
        Set0 = 1;
      }

      // Algo Text
      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.drawRectangle(6, 62, 234, 31);
      Brain.Screen.setPenColor(white);
      Brain.Screen.printAt(10 + 27, 62 + 23, Algo);

      // Probably Important
      if (Controller) {
        Controller = 0;
        LeftVelo.clear();
        LeftActualVelo.clear();
        RightVelo.clear();
        RightActualVelo.clear();
        LeftVelo.push_back(0);
        LeftActualVelo.push_back(0);
        RightVelo.push_back(0);
        RightActualVelo.push_back(0);
      }

      if (SubScreen == 0) {
        // Drawing the PurePursuit Route
        // DrawPursuitRoute(Side);
        // Add the newest Points
        // if (PurePursuit::StartPurePursuit) {
        //   LeftVelo.push_back(LimitPower(PurePursuit::LeftPower));
        //   LeftActualVelo.push_back(LimitPower(PurePursuit::LeftMeasuredVelocity));
        //   RightVelo.push_back(LimitPower(PurePursuit::RightPower));
        //   RightActualVelo.push_back(LimitPower(PurePursuit::RightMeasuredVelocity));
        // }
        
        // Drawing Every line on the Left Side
        for (int Left = 0; Left < LeftVelo.size() - 1; Left++) {
          Brain.Screen.setPenColor(ClrPink);
          Brain.Screen.drawLine(6 + ((235.0/LeftVelo.size()) * Left), 164.0 - (LeftVelo.at(Left) * (125.0/24.0)), 6 + ((235.0/LeftVelo.size()) * (Left + 1)), 164.0 - (LeftVelo.at(Left + 1) * (125.0/24.0)));
          Brain.Screen.setPenColor(red);
          Brain.Screen.drawLine(6 + ((235.0/LeftActualVelo.size()) * Left), 164.0 - (LeftActualVelo.at(Left) * (125.0/24.0)), 6 + ((235.0/LeftActualVelo.size()) * (Left + 1)), 164.0 - (LeftActualVelo.at(Left + 1) * (125.0/24.0)));
        }
        // Drawing Every line on the Right Side
        for (int Right = 0; Right < RightVelo.size() - 1; Right++) {
          Brain.Screen.setPenColor(white);
          Brain.Screen.drawLine(6 + ((235.0/RightVelo.size()) * Right), 164.0 - (RightVelo.at(Right) * (125.0/24.0)), 6 + ((235.0/RightVelo.size()) * (Right + 1)), 164.0 - (RightVelo.at(Right + 1) * (125.0/24.0)));
          Brain.Screen.setPenColor(green);
          Brain.Screen.drawLine(6 + ((235.0/RightActualVelo.size()) * Right), 164.0 - (RightActualVelo.at(Right) * (125.0/24.0)), 6 + ((235.0/RightActualVelo.size()) * (Right + 1)), 164.0 - (RightActualVelo.at(Right + 1) * (125.0/24.0)));
        }
      
      } else if (SubScreen == 1) {
        // Drawing the Boomerang Route
        // DrawBoomerang(Side);
        // Add the Newest Points
        // if (Boom::StartBoomerang) {
          // LeftVelo.push_back(LimitPower(Boom::DrivePower + Boom::TurnPower));
          // RightVelo.push_back(LimitPower(Boom::DrivePower - Boom::TurnPower));
        // }

        Brain.Screen.setPenColor(red);
        // Drawing Every line on the Left Side
        for (int Left = 0; Left < LeftVelo.size() - 1; Left++) {
          Brain.Screen.drawLine(6 + ((235.0/LeftVelo.size()) * Left), 164.0 - (LeftVelo.at(Left) * (125.0/24.0)), 6 + ((235.0/LeftVelo.size()) * (Left + 1)), 164.0 - (LeftVelo.at(Left + 1) * (125.0/24.0)));
        }
        Brain.Screen.setPenColor(green);
        // Drawing Every line on the Right Side
        for (int Right = 0; Right < RightVelo.size() - 1; Right++) {
          Brain.Screen.drawLine(6 + ((235.0/RightVelo.size()) * Right), 164.0 - (RightVelo.at(Right) * (125.0/24.0)), 6 + ((235.0/RightVelo.size()) * (Right + 1)), 164.0 - (RightVelo.at(Right + 1) * (125.0/24.0)));
        }

      }
    } else if (ScreenMode == 6) {
      // Draw Graph Back Ground
      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.drawRectangle(5, 48, 468, 186);

      Brain.Screen.setPenWidth(1);
      Brain.Screen.setPenColor("#262728");
      Brain.Screen.setFillColor("#121314");

      // Drawing the graph lines
      for (int Graph = 60; Graph < 230; Graph += 15) {
        Brain.Screen.drawLine(5, Graph, 473, Graph);
      }
      // Graph Enlarge Button Button
      if (Brain.Screen.pressing() && (XPOSI > 5 && XPOSI < 473) && (YPOSI > 48 && YPOSI < 234)) {
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor(transparent);
        Brain.Screen.drawRectangle(4, 48, 469, 187);
      } else if (!Brain.Screen.pressing() && (XPOSI > 5 && XPOSI < 473) && (YPOSI > 48 && YPOSI < 234) && ScreenMode != 1) {
        ScreenMode = 5;
        Set0 = 1;
      }

      // Algo Text
      Brain.Screen.setPenWidth(2);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor("#121314");
      Brain.Screen.drawRectangle(6, 10, 234, 31);
      Brain.Screen.drawRectangle(240, 10, 234, 31);
      Brain.Screen.setPenColor(white);

      Brain.Screen.setPenWidth(1);
      if (SubScreen == 0) {
        // Add the newest Points
        // if (PurePursuit::StartPurePursuit) {
        //   LeftVelo.push_back(LimitPower(PurePursuit::LeftPower));
        //   LeftActualVelo.push_back(LimitPower(PurePursuit::LeftMeasuredVelocity));
        //   RightVelo.push_back(LimitPower(PurePursuit::RightPower));
        //   RightActualVelo.push_back(LimitPower(PurePursuit::RightMeasuredVelocity));
        // }

        // Brain.Screen.printAt(10 + 27, 10 + 23, "%.2f / %.2f", PurePursuit::LeftPower, PurePursuit::LeftMeasuredVelocity);
        // Brain.Screen.printAt(240 + 27, 10 + 23, "%.2f / %.2f", PurePursuit::RightPower, PurePursuit::RightMeasuredVelocity);

        // double PPTing = PurePursuit::GetAccuracy();

        // Brain.Screen.setPenColor(PPTing - 20.0);
        // Brain.Screen.printAt(10 + 27, 10 + 23, "%.2f, %.2f", PPTing, 100.0);
        // Brain.Screen.setPenColor(white);
        // Brain.Screen.printAt(240 + 27, 10 + 23, "%.2f / %.2f", PurePursuit::LeftPower, PurePursuit::RightPower);

        // Drawing Every line on the Left Side
        for (int Left = 0; Left < LeftVelo.size() - 1; Left++) {
          Brain.Screen.setPenColor(ClrPink);
          Brain.Screen.drawLine(5 + ((468.0/LeftVelo.size()) * Left), 144.0 - (LeftVelo.at(Left) * (186.0/24.0)), 6 + ((468.0/LeftVelo.size()) * (Left + 1)), 144.0 - (LeftVelo.at(Left + 1) * (186.0/24.0)));
          Brain.Screen.setPenColor(red);
          Brain.Screen.drawLine(5 + ((468.0/LeftActualVelo.size()) * Left), 144.0 - (LeftActualVelo.at(Left) * (186.0/24.0)), 6 + ((468.0/LeftActualVelo.size()) * (Left + 1)), 144.0 - (LeftActualVelo.at(Left + 1) * (186.0/24.0)));
        }
        // Drawing Every line on the Right Side
        for (int Right = 0; Right < RightVelo.size() - 1; Right++) {
          Brain.Screen.setPenColor(ClrLime);
          Brain.Screen.drawLine(6 + ((468.0/RightVelo.size()) * Right), 144.0 - (RightVelo.at(Right) * (186.0/24.0)), 6 + ((468.0/RightVelo.size()) * (Right + 1)), 144.0 - (RightVelo.at(Right + 1) * (186.0/24.0)));
          Brain.Screen.setPenColor(ClrDarkGreen);
          Brain.Screen.drawLine(6 + ((468.0/RightActualVelo.size()) * Right), 144.0 - (RightActualVelo.at(Right) * (186.0/24.0)), 6 + ((468.0/RightActualVelo.size()) * (Right + 1)), 144.0 - (RightActualVelo.at(Right + 1) * (186.0/24.0)));
        }
      } else if (SubScreen == 1) {
        // Add the Newest Points
        // if (Boom::StartBoomerang) {
          // LeftVelo.push_back(LimitPower(Boom::DrivePower + Boom::TurnPower));
          // RightVelo.push_back(LimitPower(Boom::DrivePower - Boom::TurnPower));
        // }

        // Brain.Screen.printAt(10 + 27, 10 + 23, "%.2f", Boom::DrivePower + Boom::TurnPower);
        // Brain.Screen.printAt(240 + 27, 10 + 23, "%.2f", Boom::DrivePower - Boom::TurnPower);

        Brain.Screen.setPenColor(red);
        // Drawing Every line on the Left Side
        for (int Left = 0; Left < LeftVelo.size() - 1; Left++) {
          Brain.Screen.drawLine(5 + ((468.0/LeftVelo.size()) * Left), 132.0 - (LeftVelo.at(Left) * (186.0/24.0)), 6 + ((468.0/LeftVelo.size()) * (Left + 1)), 132.0 - (LeftVelo.at(Left + 1) * (186.0/24.0)));
        }
        Brain.Screen.setPenColor(green);
        // Drawing Every line on the Right Side
        for (int Right = 0; Right < RightVelo.size() - 1; Right++) {
          Brain.Screen.drawLine(6 + ((468.0/RightVelo.size()) * Right), 132.0 - (RightVelo.at(Right) * (186.0/24.0)), 6 + ((468.0/RightVelo.size()) * (Right + 1)), 132.0 - (RightVelo.at(Right + 1) * (186.0/24.0)));
        }
      }
    }
  }
}

// void Draw::DrawPursuitRoute(int Start) {
//   Brain.Screen.setPenWidth(1);
//   if (PurePursuit::NewPoints.size() > 1) {
//     if (!Start) {
//       Brain.Screen.setPenColor(black);
//       Brain.Screen.setFillColor(black);
//       for (int Point = 0; Point < PurePursuit::NewPoints.size() - 1; Point++) {
//         Brain.Screen.drawLine((PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + 3 * TileWidth) + PurePursuit::NewPoints.at(Point).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(Point).GetY() * Scale, (PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + 3 * TileWidth) + PurePursuit::NewPoints.at(Point + 1).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(Point + 1).GetY() * Scale);
//       }
//       Brain.Screen.setPenColor(green);
//       Brain.Screen.setFillColor(transparent);
//       Brain.Screen.setPenWidth(1);
//       Brain.Screen.drawCircle((PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + 3 * TileWidth) + PurePursuit::NewPoints.at(PurePursuit::NewPoints.size() - 1).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(PurePursuit::NewPoints.size() - 1).GetY() * Scale, 4);
//     } else if (Start) {
//       Brain.Screen.setPenColor(black);
//       Brain.Screen.setFillColor(black);
//       for (int Point = 0; Point < PurePursuit::NewPoints.size() - 1; Point++) {
//         Brain.Screen.drawLine((PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + TileWidth * 5 - 11) + PurePursuit::NewPoints.at(Point).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(Point).GetY() * Scale, (PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + TileWidth * 5 - 11) + PurePursuit::NewPoints.at(Point + 1).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(Point + 1).GetY() * Scale);
//       }
//       Brain.Screen.setPenColor(green);
//       Brain.Screen.setFillColor(transparent);
//       Brain.Screen.setPenWidth(1);
//       Brain.Screen.drawCircle((PurePursuit::NewPoints.at(0).GetX() * Scale + StartPositionX + TileWidth * 5 - 11) + PurePursuit::NewPoints.at(PurePursuit::NewPoints.size() - 1).GetX() * Scale, (PurePursuit::NewPoints.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth) - PurePursuit::NewPoints.at(PurePursuit::NewPoints.size() - 1).GetY() * Scale, 4);
//     }
//   }
// }

// void Draw::DrawBoomerang(int Start) {
//   // Final Vector of Points
//   std::vector<Point> FinalArc;
//   // Original Vector of Points
//   std::vector<Point> ArcPoints = {Point(MCL::X, MCL::Y), Point((MCL::X + LimitDis() * sin(MCL::Angle) + sin(MCL::ToRadians(Boom::EndAngle)) * Scale), (MCL::Y + LimitDis() * cos(MCL::Angle) + cos(MCL::ToRadians(Boom::EndAngle)) * Scale)), Point(Boom::EndX, Boom::EndY)};
//   // The Injecting and Smoothing
//   for (int X = 0; X < ArcPoints.size() - 1; X++) {
//     Point Aang(ArcPoints.at(X + 1).GetX() - ArcPoints.at(X).GetX(), ArcPoints.at(X + 1).GetY() - ArcPoints.at(X).GetY());
//     double Relativity = sqrt(pow((ArcPoints.at(X + 1).GetX() - ArcPoints.at(X).GetX()), 2) + pow((ArcPoints.at(X + 1).GetY() - ArcPoints.at(X).GetY()), 2));
//     double MorePoints = Relativity / 3;
//     Aang = Point(Aang.GetX() / MorePoints, Aang.GetY() / MorePoints);
//     for (int Injection = 0; Injection < MorePoints; Injection++) {
//       Point NewPoint((ArcPoints.at(X).GetX() + (Aang.GetX() * Injection)), (ArcPoints.at(X).GetY() + (Aang.GetY() * Injection)));
//       FinalArc.push_back(NewPoint);
//     }
//   }

//   double Tolerance = .001, b1 = .8;
//   double a1 = 1 - b1;
//   double Change = Tolerance;
//   std::vector<Point> TempPoints = FinalArc;
//   while (Change >= Tolerance) {
//     Change = 0.0;
//     for (int P = 1; P < FinalArc.size() - 1; P++) {
//       double Xthing = FinalArc.at(P).GetX();
//       double Ything = FinalArc.at(P).GetY();
//       FinalArc.at(P) = Point(FinalArc.at(P).GetX() + (a1 * (TempPoints.at(P).GetX() - FinalArc.at(P).GetX()) + b1 * (FinalArc.at(P-1).GetX() + FinalArc.at(P+1).GetX() - (2.0 * FinalArc.at(P).GetX()))), FinalArc.at(P).GetY() + (a1 * (TempPoints.at(P).GetY() - FinalArc.at(P).GetY()) + b1 * (FinalArc.at(P-1).GetY() + FinalArc.at(P+1).GetY() - (2.0 * FinalArc.at(P).GetY()))));
//       Change += fabs((Xthing - FinalArc.at(P).GetX()) + (Ything - FinalArc.at(P).GetY()));
//     }
//   }
//   FinalArc.push_back(ArcPoints.at(ArcPoints.size() - 1));

//   // All the Drawing
//   if (!Start) {
//     PredictX = MCL::X + 20 * sin(MCL::Angle) + 10 * sin(Boom::EndAngle);
//     PredictY = MCL::Y + 20 * -cos(MCL::Angle) + 10 * -cos(Boom::EndAngle);
//     // Drawing the End Point
//     Brain.Screen.setPenColor(green);
//     Brain.Screen.setPenWidth(1);
//     Brain.Screen.drawCircle(Boom::EndX * Scale + StartPositionX + 3 * TileWidth, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth, 4, black);
//     // Draw End Angle
//     Brain.Screen.setPenColor(174);
//     PredictX = (Boom::EndX * Scale + StartPositionX + 3 * TileWidth) + 24 * sin(MCL::ToRadians(Boom::EndAngle));
//     PredictY = (-Boom::EndY * Scale + StartPositionY + 3 * TileWidth) + 24 * cos(MCL::ToRadians(Boom::EndAngle));
//     Brain.Screen.drawLine(Boom::EndX * Scale + StartPositionX + 3 * TileWidth, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
//     // Draw Line through the End Point
//     Brain.Screen.setPenColor(174);
//     Brain.Screen.drawLine((MCL::X * Scale + StartPositionX + 3 * TileWidth), (-MCL::Y * Scale + StartPositionY + 3 * TileWidth), Boom::EndX * Scale + StartPositionX + 3 * TileWidth, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth);
    
//     // Real Path Aproximation
//     Brain.Screen.setPenColor(black);
//     for (int Point = 0; Point < FinalArc.size() - 1; Point++) {
//       Brain.Screen.drawLine(FinalArc.at(Point).GetX() * Scale + StartPositionX + 3 * TileWidth, -FinalArc.at(Point).GetY() * Scale + StartPositionY + 3 * TileWidth, FinalArc.at(Point + 1).GetX() * Scale + StartPositionX + 3 * TileWidth, -FinalArc.at(Point + 1).GetY() * Scale + StartPositionY + 3 * TileWidth);
//     }
//   } else if (Start) {
//     PredictX = MCL::X + 20 * sin(MCL::Angle) + 10 * sin(Boom::EndAngle);
//     PredictY = MCL::Y + 20 * -cos(MCL::Angle) + 10 * -cos(Boom::EndAngle);
//     // Drawing the End Point
//     Brain.Screen.setPenColor(green);
//     Brain.Screen.setPenWidth(1);
//     Brain.Screen.drawCircle(Boom::EndX * Scale + StartPositionX + TileWidth * 5 - 11, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth, 4, black);
//     // Draw End Angle
//     Brain.Screen.setPenColor(174);
//     PredictX = (Boom::EndX * Scale + StartPositionX + TileWidth * 5 - 11) + 24 * sin(MCL::ToRadians(Boom::EndAngle));
//     PredictY = (-Boom::EndY * Scale + StartPositionY + 3 * TileWidth) + 24 * cos(MCL::ToRadians(Boom::EndAngle));
//     Brain.Screen.drawLine(Boom::EndX * Scale + StartPositionX + TileWidth * 5 - 11, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth, PredictX, PredictY);
//     // Draw Line through the End Point
//     Brain.Screen.setPenColor(174);
//     Brain.Screen.drawLine((MCL::X * Scale + StartPositionX + TileWidth * 5 - 11), (-MCL::Y * Scale + StartPositionY + 3 * TileWidth), Boom::EndX * Scale + StartPositionX + TileWidth * 5 - 11, -Boom::EndY * Scale + StartPositionY + 3 * TileWidth);
  
//     // Real Path Aproximation
//     Brain.Screen.setPenColor(black);
//     for (int Point = 0; Point < FinalArc.size() - 1; Point++) {
//       Brain.Screen.drawLine(FinalArc.at(Point).GetX() * Scale + StartPositionX + TileWidth * 5 - 11, -FinalArc.at(Point).GetY() * Scale + StartPositionY + 3 * TileWidth, FinalArc.at(Point + 1).GetX() * Scale + StartPositionX + TileWidth * 5 - 11, -FinalArc.at(0).GetY() * Scale + StartPositionY + 3 * TileWidth);
//     }
//   }
// }

void Draw::DrawAutoRoutes(int Auton) {
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(174);
  if (Auton == 1) {
    // Brain.Screen.drawLine(StartPositionX + 12, StartPositionY + 43, StartPositionX + TileWidth * 5 + TileWidth / 4, StartPositionY + TileWidth - TileWidth / 4);

  } else if (Auton == 2) {
    
  } else if (Auton == 3) {
    
  } else if (Auton == 4) {
    
  } else if (Auton == 5) {
    
  } else if (Auton == 6) {
    
  }
}

void Draw::Cycle(void) {
  if (SubScreen == 0) {
    SubScreen = 1;
    Algo = (char*)"Algo:: Boomerang";
  } else if (SubScreen == 1) {
    SubScreen = 0;
    Algo = (char*)"Algo:: PurePursuit";
  }
}

double Draw::LimitPower(double POWER) {
  if (POWER > 12) {
    POWER = 12;
  }
  if (POWER < -12) {
    POWER = -12;
  }
  return POWER;
}

// double Draw::LimitDis(void) {
//   double lim = sqrt(pow(Boom::EndX - MCL::X, 2) + pow(Boom::EndY - MCL::Y, 2));
//   if (lim > 15) {
//     lim = 15;
//   }
//   return lim;
// }