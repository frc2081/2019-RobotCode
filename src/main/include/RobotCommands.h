/*
 * RobotCommands.h
 *
 *  Created on: Jan 26, 2018
 *      Author: blzzrd
 */

#ifndef SRC_ROBOTCOMMANDS_H_
#define SRC_ROBOTCOMMANDS_H_

class RobotCommands {
public:
	RobotCommands();

	double drvang;
	double drvmag;
	double drvrot;

	double autodrvang;
	double autodrvmag;
	double autodrvrot;
	bool guidanceSysActive;

	bool climbCommand;
	bool climbAbort;
	bool hatchPickup;
	bool cargoPickup;
	bool elevatorHome;
	bool ejectCargo;
	bool placeHatchOne;
	bool placeHatchTwo;
	bool placeCargoInShip;
	bool placeCargoRocketOne;
	bool placeCargoRocketTwo;

	bool autoHatchPickup;
	bool autoPlaceHatchOne;
	bool autoPlaceHatchTwo;
	bool autoPlaceCargoInShip;
	bool autoPlaceCargoRocketOne;
	bool autoPlaceCargoRocketTwo;

	bool hatchArmToggleManual;
	bool ballArmMotorIntakeManual;
	bool ballArmMotorEjectManual;
	bool hatchClawManual;
	bool ballEjectorManual;
	bool ballArmToggleManual;
	double elevatorDrivePowerManual;
};

#endif /* SRC_ROBOTCOMMANDS_H_ */
