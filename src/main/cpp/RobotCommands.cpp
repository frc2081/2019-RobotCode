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
	climbCommand = false;
	climbAbort = false;

	hatchPickup = false;
	cargoPickup = false;
	elevatorHome = false;
	ejectCargo = false;
	placeHatchOne = false;
	placeHatchTwo = false;
	placeCargoInShip = false;
	placeCargoRocketOne = false;
	placeCargoRocketTwo = false;

	autoHatchPickup = false;
	autoPlaceHatchOne = false;
	autoPlaceHatchTwo = false;
	autoPlaceCargoInShip = false;
	autoPlaceCargoRocketOne = false;
	autoPlaceCargoRocketTwo = false;

	hatchArmToggleManual = false;
	ballArmMotorIntakeManual = false;
	ballArmMotorEjectManual = false;
	hatchClawManual = false;
	ballEjectorManual = false;
	ballArmToggleManual = false;
	elevatorDrivePowerManual = 0;

}

