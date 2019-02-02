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

	autoHatchPickup = false;
	autoPlaceHatchOne = false;
	autoPlaceHatchTwo = false;
	autoPlaceCargoInShip = false;
	autoPlaceCargoRocketOne = false;
	autoPlaceCargoRocketTwo = false;

}

