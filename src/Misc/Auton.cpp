// Default
#include "vex.h"
#include "cmath"
#include "api.h"

// Namespace
namespace Auton { 
  bool FirstPath = false;
  void RunAuton(void);
  void CalcFirstPath(void);
}

void PauseOnRing(void) {
  thread([]{
    while (Dis.value() > 35) {
      this_thread::sleep_for(1);
    } 
    Intake.stop(coast);
    // thread([]{
    //   this_thread::sleep_for(400);
    //   double startPos = Intake.position(rev);
    //   Intake.spin(reverse, 3, volt);
    //   waitUntil(.9 - (startPos - Intake.position(rev)) < .1);
    //   Intake.stop(brake);
    // });
  });
}

void ScoreGoal(void) {
//   IntakeState = 1;
  Intake.spin(reverse, 12, volt);
  // wait(250, msec);
  Clamp.close();
  wait(250, msec);
  // IntakeState = 0;
  Intake.spin(fwd, 12, volt);
}

void Reset(void) {
  thread([]{
    this_thread::sleep_for(400);

    Lift.spin(reverse, 12, volt);
    this_thread::sleep_for(600);
    // LadyBrown::StartLadyBrown(10);
    // thread([]{
    //   while (fabs(LadyBrown::target - Lift.position(deg)) > 5) {
        // this_thread::sleep_for(600);
    //   }
    // });
    // LadyBrown::startLadyBrownMP = false;
    Lift.stop(coast);
  });
}

void ScoreWallStake(void) {
  LadyBrown::startLadyBrownMP = false;
  Intake.stop(coast);
  wait(150, msec);
  Lift.spin(fwd, 12, volt);
  wait(500, msec);
  Lift.stop(coast);
  wait(350, msec);
}

void Auton::RunAuton(void) {
  Col.setLightPower(100);
  timer MonkeTimer;

  thread AutoClampThread( [] {
    bool AutoClampState = false;
    // timer ClampTimer;
    while(true) {
      if (AutoClamp.value() < 90 && !AutoClampState) {
        Clamp.open();
        AutoClampState = true;
        Controller1.rumble(".");
      }

      if (AutoClamp.value() > 100 && AutoClampState) {
        AutoClampState = false;
      }
      this_thread::sleep_for(5);
    }
  });

/*--------------------------------------------------------------------------*/
/*                            RED Side Autos                                */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                     Red Ring Side, Ring Rush AWP                         */
/*--------------------------------------------------------------------------*/

if (autonType == 1) {
  Lift.setPosition(36, deg);
  ColorType = RED;
  MCL::StartMCL(-7, -66, 120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(-21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Rush the Rings */
  MCL::FrontTurn(-48, -15, 8, 2, false);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(MCL::X, MCL::Y), Point(-46, -15), /* Point(-63, -15), */ Point(-68, -14)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  // Grab the Goal
  Boom::MoveToPoint(-36, -24, 12, 9, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }


  Boom::MoveToPoint(-57, -24, 9.6, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(-51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(-57, -30), Point(-57, -57), Point(-90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  thread([]{
    while(MCL::X > -58) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-12, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(10, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-10, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  MCL::FrontTurn(0, 0, 6, 2);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Boom::MoveToPoint(-16, -16, 9.6, 0);
  wait(900, msec);
  Boom::MaxVelo = 3.2;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}

/*---------------------------------------------------------------------*/
/*                     Red Ring Side, Solo AWP                         */
/*---------------------------------------------------------------------*/

if (autonType == 2) {
  Lift.setPosition(36, deg);
  ColorType = RED;
  MCL::StartMCL(-7, -66, 120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(-21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Grab the Stack */
  Boom::MoveToPoint(-51, -30, 12, 0); // was -36
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }


  // Boom::MoveToPoint(-24, -30, 12, 0, true);
  // wait(200, msec);

  MCL::FrontTurn(-51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(-51, -30), Point(-51, -51), Point(-90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();

  // Start the Thread
  thread([]{
    while(MCL::X > -58) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-6, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }





  MCL::FrontTurn(24, -48, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(MCL::X, MCL::Y), Point(24, -54)});
  PurePursuit::SetPath(3, 12, 2, 14, 1, false);
  // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();

  // Start the Thread for 
  thread([]{
    while (MCL::X < -24) {
      this_thread::sleep_for(1);
    }
    ScoreGoal();
    Intake.spin(reverse, 12, volt);
  });
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  MCL::BackTurn(24, -24, 6, 2, true);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Intake.spin(fwd, 12, volt);

  Boom::MoveToPoint(24, -21, 12, 0, true);
  wait(500, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Clamp the Goal
  Clamp.open(); Intake.spin(fwd, 12, volt);

  // Grab the Stack
  Boom::MoveToPoint(54, -30, 12, 0);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  Boom::MoveToPoint(24, -24, 12, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(0, 0, 6, 2);
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Boom::MoveToPoint(16, -16, 12, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}

/*------------------------------------------------------------------------*/
/*                   Red Goal Side, Safe Half AWP                         */
/*------------------------------------------------------------------------*/

if (autonType == 3) {
  Lift.setPosition(36, deg);
  ColorType = RED;
  MCL::StartMCL(7, -66, -120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Get the Stack */
  Boom::MoveToPoint(51, -30, 12, 0);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(51, -30), Point(51, -51), Point(90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  thread([]{
    while(MCL::Y > -55) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-12, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(400, msec);

  DrivePID::DrivePidOn(10, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(500, msec);

  DrivePID::DrivePidOn(-10, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(600, msec);

  MCL::FrontTurn(0, 0, 6, 2);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  Boom::MoveToPoint(16, -16, 9.6, 0);
  wait(900, msec);
  Boom::MaxVelo = 3.2;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}

/*--------------------------------------------------------------------------*/
/*                        Red Goal Side, Goal Rush                          */
/*--------------------------------------------------------------------------*/

if (autonType == 4) {
  Lift.setPosition(10, deg);
  ColorType = RED;
  MCL::StartMCL(60, -48, -20);

  // Lol
  Intake.stop(coast);

  // Open Goal Rush
  Doinker.open();

  // Set the Intake Front
  PurePursuit::SetIntakeFront();

  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(MCL::X, MCL::Y), /* Point(19, 32), */ Point(48, 0)});
  PurePursuit::SetPath(3, 12, 12, 16, 1, true);
  // Wait until done
  PurePursuit::StartPath();
  thread([]{
    // while(LadyBrown::) {
    //   this_thread::sleep_for(1);
    // }
    this_thread::sleep_for(200);
    Intake.spin(fwd, 6, volt);
    PauseOnRing();
  });

  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
    if (DoinkerSwitch.value() == 1 && MCL::Y > -36) {
      Doinker.close();
      // PurePursuit::StartPurePursuit = false;
      PurePursuit::TimedStop(1);
      break;
      // wait(50,msec);
    }
  }
  Doinker.close();

  // return;

  Boom::MoveToPoint(-60, -48, 12, 4, true);

  thread([]{
    while(MCL::Y > -36) {
      this_thread::sleep_for(1);
    }
    Doinker.open();
  });
  while(Boom::StartBoomerang){
    wait(1, msec);
  }

  // DrivePID::DrivePidOn(-6, 12);
  // while(DrivePID::startDrivePID) {
  //   wait(1, msec);
  // }

  Doinker.close();

  return;

  // Doinker.open();
}

/*---------------------------------------------------------------------------*/
/*                        Blue Side Autos                                    */
/*---------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                     Blue Ring Side, Ring Rush AWP                        */
/*--------------------------------------------------------------------------*/

if (autonType == 5) {
  Lift.setPosition(36, deg);
  ColorType = BLUE;
  MCL::StartMCL(7, -66, -120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Rush the Rings */
  MCL::FrontTurn(48, -15, 8, 2, false);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(MCL::X, MCL::Y), Point(46, -15), /* Point(63, -15), */ Point(68, -14)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  // Grab the Goal
  Boom::MoveToPoint(36, -24, 12, 9, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }


  Boom::MoveToPoint(57, -24, 9.6, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(57, -30), Point(57, -57), Point(90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  thread([]{
    while(MCL::X < 58) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-12, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(10, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-10, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  MCL::FrontTurn(0, 0, 6, 2);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Boom::MoveToPoint(16, -16, 9.6, 0);
  wait(900, msec);
  Boom::MaxVelo = 3.2;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}

/*---------------------------------------------------------------------*/
/*                     Blue Ring Side, Solo AWP                        */
/*---------------------------------------------------------------------*/

if (autonType == 6) {
  Lift.setPosition(36, deg);
  ColorType = BLUE;
  MCL::StartMCL(7, -66, -120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Grab the Stack */
  Boom::MoveToPoint(51, -30, 12, 0); // was -36
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }


  // Boom::MoveToPoint(-24, -30, 12, 0, true);
  // wait(200, msec);

  MCL::FrontTurn(51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(51, -30), Point(51, -51), Point(90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();

  // Start the Thread
  thread([]{
    while(MCL::X < 58) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-6, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }





  MCL::FrontTurn(-24, -48, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(MCL::X, MCL::Y), Point(-24, -54)});
  PurePursuit::SetPath(3, 12, 2, 14, 1, false);
  // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();

  // Start the Thread for 
  thread([]{
    while (MCL::X > 24) {
      this_thread::sleep_for(1);
    }
    ScoreGoal();
    Intake.spin(reverse, 12, volt);
  });
  
  // Wait until done
  PurePursuit::StartPath();
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  MCL::BackTurn(-24, -24, 6, 2, true);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Intake.spin(fwd, 12, volt);

  Boom::MoveToPoint(-24, -21, 12, 0, true);
  wait(500, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Clamp the Goal
  Clamp.open(); Intake.spin(fwd, 12, volt);

  // Grab the Stack
  Boom::MoveToPoint(-54, -30, 12, 0);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  Boom::MoveToPoint(-24, -24, 12, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(0, 0, 6, 2);
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  Boom::MoveToPoint(-16, -16, 12, 0, true);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}

/*------------------------------------------------------------------------*/
/*                   Red Goal Side, Safe Half AWP                         */
/*------------------------------------------------------------------------*/

if (autonType == 7) {
  Lift.setPosition(36, deg);
  ColorType = BLUE;
  MCL::StartMCL(-7, -66, 120);

  // Prep the LB
  Intake.spin(fwd, 12, volt);
  LadyBrown::LadyBrownOne(); wait(50, msec);
  waitUntil(Intake.velocity(rpm) < 5); wait(60, msec);
  Intake.stop(coast);

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(375, msec);
  Lift.stop(coast);
  Intake.stop(coast);

  // Reset the LadyBrown
  Reset();

  // Grab the Goal
  Boom::MoveToPoint(-21, -24, 12, 0, true);
  wait(800, msec);
  Boom::MaxVelo = 6;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Clamp the Goal
  Clamp.open();

  /* Get the Stack */
  Boom::MoveToPoint(-51, -30, 12, 0);
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  MCL::FrontTurn(-51, -51, 6, 2, true);
  // Calc Next Path
  PurePursuit::AddPath(std::vector<Point> {Point(-51, -30), Point(-51, -51), Point(-90, -95)});
  PurePursuit::SetPath(3, 12, 0, 14, 1, false);
  // // wait Until Finished
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  // Set the Intake Front
  PurePursuit::SetIntakeFront();
  
  // Wait until done
  PurePursuit::StartPath();
  thread([]{
    while(MCL::Y > -55) {
      this_thread::sleep_for(1);
    }
    PurePursuit::MaxSpeed = 3.3;
  });
  while(PurePursuit::StartPurePursuit) {
    wait(1, msec);
  }

  DrivePID::DrivePidOn(-12, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(400, msec);

  DrivePID::DrivePidOn(10, 8);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(500, msec);

  DrivePID::DrivePidOn(-10, 12);
  while(DrivePID::startDrivePID) {
    wait(1, msec);
  }

  wait(600, msec);

  MCL::FrontTurn(0, 0, 6, 2);
  Intake.stop(coast); LadyBrown::LadyBrownHold();
  while(TurnPID::startTurnPID) {
    wait(1, msec);
  }

  // Intake
  Intake.spin(fwd, 12, volt);

  Boom::MoveToPoint(-16, -16, 9.6, 0);
  wait(900, msec);
  Boom::MaxVelo = 3.2;
  while(Boom::StartBoomerang) {
    wait(1, msec);
  }

  // Lol
  Lift.spin(fwd, 12, volt);
  wait(600, msec);
  Lift.stop(coast);
}


/*---------------------------------------------------------------------------*/
/*                        Blue Goal Side, Goal Rush                          */
/*---------------------------------------------------------------------------*/

if (autonType == 8) {
  
}

/*---------------------------------------*/
/*              Testing/Empty            */
/*---------------------------------------*/

if (autonType == -1) {
  
}

/*---------------------------------------------------------------------------*/
/*                              End of Auton's                               */
/*---------------------------------------------------------------------------*/

  cout << "AUTON TIME: " << MonkeTimer.time(msec) << endl;
}




void Auton::CalcFirstPath(void) {

  FirstPath = true;

  // PurePursuit::PrimaryPoints.clear();
  // PurePursuit::NewPoints.clear();
  // PurePursuit::PathPoints.clear();

/*---------------------------------------------------------------------------*/
/*                         Left side, 4/5 Triball                            */
/*---------------------------------------------------------------------------*/

  if (autonType == 1 || autonType == 2 || autonType == 8 || autonType == 9 || autonType == 12 || autonType == 13) {
    
  }

/*---------------------------------------------------------------------------*/
/*                     Left side, 5 Triball Rush                             */
/*---------------------------------------------------------------------------*/

  if (autonType == 3) {
    
  }

/*---------------------------------------------------------------------------*/
/*                         Left side, Disrupter Auto                         */
/*---------------------------------------------------------------------------*/

  if (autonType == 4 || autonType == 10 || autonType == 14) {
    
  }

/*---------------------------------------------------------------------------*/
/*                        Right side, 5 Triball Rush                         */
/*---------------------------------------------------------------------------*/

  if (autonType == 5) {
    
  }

/*---------------------------------------------------------------------------*/
/*                        Right side, 6 Triball Rush                         */
/*---------------------------------------------------------------------------*/

  if (autonType == 6 || autonType == 5) {
    
  }

/*---------------------------------------------------------------------------*/
/*                        Right side, 6 Triball China                        */
/*---------------------------------------------------------------------------*/

  if (autonType == 7) {

  }

/*----------------------------------*/
/*              Testing             */
/*----------------------------------*/

  if (autonType == -1) {
    
  }

/*---------------------------------------------------------------------------*/
/*                              End of Auton's                               */
/*---------------------------------------------------------------------------*/

}