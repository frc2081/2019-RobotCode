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

	//extend climbing stilts/send climb command
	if (drivecontroller->bBack->State() && drivecontroller->bX->State()) Commands->climbCommandLevelOne = true;
	else Commands->climbCommandLevelOne = false;
	frc::SmartDashboard::PutBoolean("climbCommandL1", Commands->climbCommandLevelOne);
	
	if (drivecontroller->bBack->State() && drivecontroller->bY->State()) Commands->climbCommandLevelTwo = true;
	else Commands->climbCommandLevelTwo = false;
	frc::SmartDashboard::PutBoolean("climbCommandL2", Commands->climbCommandLevelTwo);	
	
	//stop climbing/abort climb command
	if (drivecontroller->bBack->State() && drivecontroller->bB->State()) Commands->climbAbort = true;
	else Commands->climbAbort = false;
	frc::SmartDashboard::PutBoolean("climbAbort", Commands->climbAbort);

	if (drivecontroller->bBack->State() && drivecontroller->bA->State()) Commands->climbFreeze = true;
	else Commands->climbFreeze= false;
	frc::SmartDashboard::PutBoolean("climbFreeze", Commands->climbFreeze);

	Commands->hatchPickup = false;
	Commands->cargoPickup = false;
	Commands->elevatorHome = false;
	Commands->ejectCargo = false;
	Commands->placeHatchOne = false;
	Commands->placeHatchTwo = false;
	Commands->placeCargoInShip = false;
	Commands->placeCargoRocketOne = false;
	Commands->placeCargoRocketTwo = false;

	Commands->autoHatchPickup = false;
	Commands->autoPlaceHatchOne = false;
	Commands->autoPlaceHatchTwo = false;
	Commands->autoPlaceCargoInShip = false;
	Commands->autoPlaceCargoRocketOne = false;
	Commands->autoPlaceCargoRocketTwo = false;

	Commands->hatchArmToggleManual = false;
	Commands->ballArmMotorIntakeManual = false;
	Commands->ballArmMotorEjectManual = false;
	Commands->hatchClawManual = false;
	Commands->frameStandManual = false;
	Commands->ballArmToggleManual = false;
	Commands->elevatorDrivePowerManual = 0;

	frc::SmartDashboard::PutNumber("RTrig", mechanismcontroller->RTrig);
	frc::SmartDashboard::PutNumber("LTrig", mechanismcontroller->LTrig);

	if(!mechanismcontroller->bLS->State() && !mechanismcontroller->LTrig > .5 && !mechanismcontroller->RTrig > .5){
		//grab hatch from hatch loading station
		if (mechanismcontroller->bRB->State()) Commands->autoHatchPickup = true;
		else Commands->autoHatchPickup = false;
		frc::SmartDashboard::PutBoolean("autoHatchPickup", Commands->autoHatchPickup);

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
		if (mechanismcontroller->bA->State()) Commands->autoPlaceHatchOne = true;
		else Commands->autoPlaceHatchOne = false;
		frc::SmartDashboard::PutBoolean("autoPlaceHatchOne", Commands->autoPlaceHatchOne);

		//place hatch on rocket level two
		if (mechanismcontroller->bB->State()) Commands->autoPlaceHatchTwo = true;
		else Commands->autoPlaceHatchTwo = false;
		frc::SmartDashboard::PutBoolean("autoPlaceHatchTwo", Commands->autoPlaceHatchTwo);

		//place cargo in cargo ship
		if (mechanismcontroller->bStart->State()) Commands->autoPlaceCargoInShip = true;
		else Commands->autoPlaceCargoInShip = false;
		frc::SmartDashboard::PutBoolean("autoPlaceCargoInShip", Commands->autoPlaceCargoInShip);

		//place cargo on rocket level one
		if (mechanismcontroller->bX->State()) Commands->autoPlaceCargoRocketOne = true;
		else Commands->autoPlaceCargoRocketOne = false;
		frc::SmartDashboard::PutBoolean("autoPlaceCargoRocketOne", Commands->autoPlaceCargoRocketOne);

		//place cargo on rocket level two
		if (mechanismcontroller->bY->State()) Commands->autoPlaceCargoRocketTwo = true;
		else Commands->autoPlaceCargoRocketTwo = false;
		frc::SmartDashboard::PutBoolean("autoPlaceCargoRocketTwo", Commands->autoPlaceCargoRocketTwo);
	}

	if(mechanismcontroller->bLS->State()){
		//grab hatch from hatch loading station
		if (mechanismcontroller->bRB->RE()) Commands->hatchPickup = true;
		else Commands->hatchPickup = false;
		frc::SmartDashboard::PutBoolean("hatchPickup", Commands->hatchPickup);

		//place hatch on rocket level one
		if (mechanismcontroller->bA->RE()) Commands->placeHatchOne = true;
		else Commands->placeHatchOne = false;
		frc::SmartDashboard::PutBoolean("placeHatchOne", Commands->placeHatchOne);

		//place hatch on rocket level two
		if (mechanismcontroller->bB->RE()) Commands->placeHatchTwo = true;
		else Commands->placeHatchTwo = false;
		frc::SmartDashboard::PutBoolean("placeHatchTwo", Commands->placeHatchTwo);

		//place cargo in cargo ship
		if (mechanismcontroller->bStart->RE()) Commands->placeCargoInShip = true;
		else Commands->placeCargoInShip = false;
		frc::SmartDashboard::PutBoolean("placeCargoInShip", Commands->placeCargoInShip);

		//place cargo on rocket level one
		if (mechanismcontroller->bX->RE()) Commands->placeCargoRocketOne = true;
		else Commands->placeCargoRocketOne = false;
		frc::SmartDashboard::PutBoolean("placeCargoRocketOne", Commands->placeCargoRocketOne);

		//place cargo on rocket level two
		if (mechanismcontroller->bY->RE()) Commands->placeCargoRocketTwo = true;
		else Commands->placeCargoRocketTwo = false;
		frc::SmartDashboard::PutBoolean("placeCargoRocketTwo", Commands->placeCargoRocketTwo);
	}

	if(mechanismcontroller->LTrig > .5 && mechanismcontroller->RTrig > .5){
		//hatch arm toggle manual
		if (mechanismcontroller->bA->RE()) Commands->hatchArmToggleManual = true;
		else Commands->hatchArmToggleManual = false;
		frc::SmartDashboard::PutBoolean("hatchArmToggleManual", Commands->hatchArmToggleManual);

		//ball arm motor intake manual
		if (mechanismcontroller->bX->RE()) Commands->ballArmMotorIntakeManual = true;
		else Commands->ballArmMotorIntakeManual = false;
		frc::SmartDashboard::PutBoolean("ballArmMotorIntakeManual", Commands->ballArmMotorIntakeManual);

		//ball arm eject manual
		if (mechanismcontroller->bY->RE()) Commands->ballArmMotorEjectManual = true;
		else Commands->ballArmMotorEjectManual = false;
		frc::SmartDashboard::PutBoolean("ballArmMotorEjectManual", Commands->ballArmMotorEjectManual);

		//open/close hatch claw manual
		if (mechanismcontroller->bBack->RE()) Commands->hatchClawManual = true;
		else Commands->hatchClawManual = false;
		frc::SmartDashboard::PutBoolean("hatchClawManual", Commands->hatchClawManual);

		//eject ball manual
		if (mechanismcontroller->bStart->RE()) Commands->frameStandManual = true;
		else Commands->frameStandManual = false;
		frc::SmartDashboard::PutBoolean("ballEjectorManual", Commands->frameStandManual);

		//toggle ball arm as up or down manual
		if (mechanismcontroller->bB->RE()) Commands->ballArmToggleManual = true;
		else Commands->ballArmToggleManual = false;
		frc::SmartDashboard::PutBoolean("ballArmToggleManual", Commands->ballArmToggleManual);

		//move elevator up or down manual
		Commands->elevatorDrivePowerManual = mechanismcontroller->LY;
		frc::SmartDashboard::PutNumber("elevator", Commands->elevatorDrivePowerManual);
	}
}