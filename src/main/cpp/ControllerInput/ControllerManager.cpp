/*
 * ControllerManager.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: blzzrd
 */

#include <ControllerManager.h>
#include <math.h>



ControllerManager::ControllerManager() {
	drivecontroller = new cntl::cntl(drivecontrollernumber, drivecontrollerdeadband, drivecontrollerupperlimit);
}

void ControllerManager::pollControllers(RobotCommands *Commands){

	drivecontroller->UpdateCntl();

	//Drive Commands
	Commands->drvang = (atan2(-drivecontroller->LX, drivecontroller->LY) * 180/3.14159265);
	Commands->drvmag = sqrt(pow(drivecontroller->LX, 2) + pow(drivecontroller->LY, 2));
	Commands->drvrot = drivecontroller->RX;

	//Swerve Drive Commands
	frc::SmartDashboard::PutNumber("Swerve Angle Command", Commands->drvang);
	frc::SmartDashboard::PutNumber("Swerve Speed Command", Commands->drvmag);
	frc::SmartDashboard::PutNumber("Swerve Rotation Command", Commands->drvrot);
}


