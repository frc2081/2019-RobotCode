/*
 * ControllerManager.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: blzzrd
 */

#include <ControllerManager.h>
#include <math.h>
#include "frc/WPILib.h"



ControllerManager::ControllerManager() {
	drivecontroller = new cntl::cntl(drivecontrollernumber, drivecontrollerdeadband, drivecontrollerupperlimit);
	mechanismcontroller= new cntl::cntl(mechanismcontrollernumber, mechanismcontrollerdeadband, mechanismcontrollerupperlimit);
}

void ControllerManager::pollControllers(RobotCommands *Commands){

	drivecontroller->UpdateCntl();
	mechanismcontroller->UpdateCntl();

	//Drive Commands
	Commands->drvang = (atan2(-drivecontroller->LX, drivecontroller->LY) * 180/3.14159265);
	Commands->drvmag = sqrt(pow(drivecontroller->LX, 2) + pow(drivecontroller->LY, 2));
	Commands->drvrot = drivecontroller->RX;

	//Swerve Drive Commands
	frc::SmartDashboard::PutNumber("Swerve Angle Command", Commands->drvang);
	frc::SmartDashboard::PutNumber("Swerve Speed Command", Commands->drvmag);
	frc::SmartDashboard::PutNumber("Swerve Rotation Command", Commands->drvrot);


//commands are the "automatic" version of command, unless specidied as "manual"

	//grab hatch from hatch loading station
	if (mechanismcontroller->bRB->State()) Commands->hatchPickup = true;
	else Commands->hatchPickup = false;
	frc::SmartDashboard::PutBoolean("hatchPickup", Commands->hatchPickup);

	//grab cargo from ground
	if (mechanismcontroller->bLB->State()) Commands->cargoPickup = true;
	else Commands->cargoPickup = false;
	frc::SmartDashboard::PutBoolean("cargoPickup", Commands->cargoPickup);

	//move elevator to home
	if (mechanismcontroller->bBack->State()) Commands->elevatorHome = true;
	else Commands->elevatorHome = false;
	frc::SmartDashboard::PutBoolean("elevatorHome", Commands->elevatorHome);

	//eject cargo
	if (mechanismcontroller->bRS->State()) Commands->ejectCargo = true;
	else Commands->ejectCargo = false;
	frc::SmartDashboard::PutBoolean("ejectCargo", Commands->ejectCargo);

	//place hatch on rocket level one
	if (mechanismcontroller->bA->State()) Commands->placeHatchOne = true;
	else Commands->placeHatchOne = false;
	frc::SmartDashboard::PutBoolean("placeHatchOne", Commands->placeHatchOne);

	//place hatch on rocket level two
	if (mechanismcontroller->bB->State()) Commands->placeHatchTwo = true;
	else Commands->placeHatchTwo = false;
	frc::SmartDashboard::PutBoolean("placeHatchTwo", Commands->placeHatchTwo);

	//place crag in cargo ship
	if (mechanismcontroller->bStart->State()) Commands->placeCargoInShip = true;
	else Commands->placeCargoInShip = false;
	frc::SmartDashboard::PutBoolean("placeCargoInShip", Commands->placeCargoInShip);

	//place cargo on rocket level one
	if (mechanismcontroller->bX->State()) Commands->placeCargoRocketOne = true;
	else Commands->placeCargoRocketOne = false;
	frc::SmartDashboard::PutBoolean("placeCargoRocketOne", Commands->placeCargoRocketOne);

	//place cargo on rocket level two
	if (mechanismcontroller->bY->State()) Commands->placeCargoRocketTwo = true;
	else Commands->placeCargoRocketTwo = false;
	frc::SmartDashboard::PutBoolean("placeCargoRocketTwo", Commands->placeCargoRocketTwo);

	//hatch arm toggle manual
	if (mechanismcontroller->bA->State()) Commands->hatchArmToggleManual = true;
	else Commands->hatchArmToggleManual = false;
	frc::SmartDashboard::PutBoolean("hatchArmToggleManual", Commands->hatchArmToggleManual);

	//ball arm motor intake manual
	if (mechanismcontroller->bX->State()) Commands->ballArmMotorIntakeManual = true;
	else Commands->ballArmMotorIntakeManual = false;
	frc::SmartDashboard::PutBoolean("ballArmMotorIntakeManual", Commands->ballArmMotorIntakeManual);

	//ball arm eject manual
	if (mechanismcontroller->bY->State()) Commands->ballArmMotorEjectManual = true;
	else Commands->ballArmMotorEjectManual = false;
	frc::SmartDashboard::PutBoolean("ballArmMotorEjectManual", Commands->ballArmMotorEjectManual);

	//open/close hatch claw manual
	if (mechanismcontroller->bBack->State()) Commands->hatchClawManual = true;
	else Commands->hatchClawManual = false;
	frc::SmartDashboard::PutBoolean("hatchClawManual", Commands->hatchClawManual);

	//eject ball manual
	if (mechanismcontroller->bStart->State()) Commands->ballEjectorManual = true;
	else Commands->ballEjectorManual = false;
	frc::SmartDashboard::PutBoolean("ballEjectorManual", Commands->ballEjectorManual);

	//toggle ball arm as up or down manual
	if (mechanismcontroller->bB->State()) Commands->ballArmToggleManual = true;
	else Commands->ballArmToggleManual = false;
	frc::SmartDashboard::PutBoolean("ballArmToggleManual", Commands->ballArmToggleManual);

	//pick up hatch manual
	if (mechanismcontroller->bRB->State()) Commands->hatchPickup = true;
	else Commands->hatchPickup = false;
	frc::SmartDashboard::PutBoolean("hatchPickup", Commands->hatchPickup);

	//move elevator up or down manual
	Commands->elevatorDrivePowerManual = mechanismcontroller->LY;
	frc::SmartDashboard::PutNumber("elevator", Commands->elevatorDrivePowerManual);
}