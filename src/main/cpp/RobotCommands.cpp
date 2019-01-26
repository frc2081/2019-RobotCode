/*
 * RobotCommands.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: blzzrd
 */

#include <RobotCommands.h>

RobotCommands::RobotCommands() {
	drvmag = 0;
	drvang = 0;
	drvrot = 0;

	hatchPickup = false;
	cargoPickup = false;
	elevatorHome = false;
	ejectCargo = false;
	placeHatchOne = false;
	placeHatchTwo = false;
	placeCargoInShip = false;
	placeCargoRocketOne = false;
	placeCargoRocketTwo = false;

	hatchArmToggleManual = false;
	ballArmMotorIntakeManual = false;
	ballArmMotorEjectManual = false;
	hatchClawManual = false;
	ballEjectorManual = false;
	ballArmToggleManual = false;
	elevatorDrivePowerManual = 0;

}



