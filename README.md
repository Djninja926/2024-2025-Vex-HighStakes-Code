<p align="center">
  <img src="logo.PNG" alt="drawing" width="400" />
</p>

<h1 align="center">High Stakes</h1>

  <p align="center">
    VRC Team 1082R - "Robot Go Vroom Vroom"
    <br/>
    Retired HS VEX team based in Lucas, Texas
    <br/>
    Part of Lovejoy Robotics Club
    <br/>
    <a href="https://www.robotevents.com/teams/VRC/1082R">RobotEvents</a>
  </p>
</div>

<br/>
<br/>
<h1 align="center">2024-2025 High Stakes Code</h1>

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=flat&logo=c%2B%2B&logoColor=white) ![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)


This repository contains the VEX V5 code for VRC Team 1082R, "Robot Go Vroom Vroom," for the 2024-2025 competition season, themed "High Stakes." The code is developed for a VEX V5 robot and includes various modules for robot control, motion planning, and localization.

<p align="center">
  <img src="high_stakes.png"/>
</p>

## Project Structure

The project is organized into several key directories:

- `/include`: Contains header files for various modules and VEX V5 API.
- `/src`: Contains source code files for the robot's functionality.
- `/vex`: Contains VEX V5-specific build system files.
- `/Localization`: Contains headers and source files related to robot localization (Kalman Filter and MCL).
- `/Misc`: Contains headers and source files for miscellaneous utilities (Auton routines, Drawing, Matrix operations, Point structures, Spline generation).
- `/Motion`: Contains headers and source files for motion planning and control (2DMP, LadyBrown, MPC, PID).

## Key Components

### Robot Configuration

- `/include/robot-config.h`: Defines the robot's hardware configuration.
- `/src/robot-config.cpp`: Implements the robot's hardware configuration.

### Core Robot Logic

- `/src/main.cpp`: The main entry point for the robot's code.

### Localization

- `/include/Localization/Kalman.h` and `/src/Localization/Kalman.cpp`: Implementation of a Kalman Filter for state estimation.
- `/include/Localization/MCL.h` and `/src/Localization/MCL.cpp`: Implementation of Monte Carlo Localization (MCL).

### Motion Control and Planning

- This repository includes various approaches to motion control and planning, such as PID control, Model Predictive Control (MPC), and potentially custom algorithms like "LadyBrown" and "2DMP."

### Miscellaneous Utilities

- Helper functions and classes for autonomous routines, drawing on the V5 screen, matrix operations, point representations, and spline generation for smooth paths.
