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
    Guidance = new GuidanceSystem(Commands);

		Drivetrain->DriveManagerInit();
    Climber->ClimbManagerInit();

}

void Robot::RobotPeriodic() {
  DriverControls->pollControllers(Commands);
  Drivetrain->UpdateDashboard();
  Climber->ClimbManagerRobotPeriodic();
  RioIO->ioRobotPeriodic();
  Guidance->GuidanceSystemRobotPeriodic();

  frc::SmartDashboard::PutNumber("Climber RF Actual Position", RioIO->liftrfenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber LF Actual Position", RioIO->liftlfenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber LB Actual Position", RioIO->liftlbenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber RB Actual Position", RioIO->liftrbenc->GetDistance());
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {
  enabledPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  enabledPeriodic();
}

void Robot::DisabledPeriodic(){
  Drivetrain->DriveManagerDisabled();
}

void Robot::TestPeriodic() {}

void Robot::enabledPeriodic() {
  Drivetrain->DriveManagerPeriodic();
  Elevator->ElevatorManagerPeriodic();
  Elevator->ElevatorManagerMechanism(RioIO);
  RioIO->ioPeriodic();
  Climber->ClimbManagerTeleopPeriodic();
  Guidance->GuidanceSystemPeriodic();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
