/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "IO.h"
#include "ControllerManager.h"
#include "DriveManager.h"
#include "RobotCommands.h"
#include "ClimbManager.h"
#include "ElevatorManager.h"
#include "GuidanceSystem.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledPeriodic() override;

 private:
  IO *RioIO;
  ControllerManager *DriverControls;
  RobotCommands *Commands;
  DriveManager *Drivetrain;
  ClimbManager *Climber;
  ElevatorManager *Elevator;
  GuidanceSystem *Guidance;

  void enabledPeriodic();
};
