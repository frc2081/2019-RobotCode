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
    Elevator = new ElevatorManager(RioIO, Commands);

		Drivetrain->DriveManagerInit();
}

void Robot::RobotPeriodic() {
  DriverControls->pollControllers(Commands);
  Drivetrain->UpdateDashboard();
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {
    RioIO->ioPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  Drivetrain->DriveManagerPeriodic();
  Elevator->ElevatorManagerPeriodic();
  Elevator->ElevatorManagerMechanism(RioIO);
  RioIO->ioPeriodic();
}

void Robot::DisabledPeriodic(){
  Drivetrain->DriveManagerDisabled();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
