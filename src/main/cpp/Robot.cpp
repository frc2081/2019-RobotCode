/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {	  
		RioIO = new IO();
		DriverControls = new ControllerManager();
		Commands = new RobotCommands();
		Drivetrain = new DriveManager(RioIO, Commands, DriverControls);
    Climber = new ClimbManager(RioIO, Commands);
    Elevator = new ElevatorManager(RioIO, Commands);

		Drivetrain->DriveManagerInit();
}

void Robot::RobotPeriodic() {
  RioIO->pollIO();
  DriverControls->pollControllers(Commands);
  Drivetrain->UpdateDashboard();
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {
  Drivetrain->DriveManagerPeriodic();
  Elevator->ElevatorManagerPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  Drivetrain->DriveManagerPeriodic();
  Elevator->ElevatorManagerPeriodic();
}

void Robot::DisabledPeriodic(){
  Drivetrain->DriveManagerDisabled();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
