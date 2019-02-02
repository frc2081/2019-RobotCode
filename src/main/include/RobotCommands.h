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
	bool climbCommand;
	bool climbAbort;
};

#endif /* SRC_ROBOTCOMMANDS_H_ */
