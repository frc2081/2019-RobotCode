/*
 * ControllerManager.h
 *
 *  Created on: Jan 26, 2018
 *      Author: blzzrd
 */

#ifndef SRC_CONTROLLERINPUT_CONTROLLERMANAGER_H_
#define SRC_CONTROLLERINPUT_CONTROLLERMANAGER_H_

#include "cntl.h"
#include "ControllerSettings.h"
#include "RobotCommands.h"



class ControllerManager {
public:
	ControllerManager();

	void pollControllers(RobotCommands *Commands);

private:
	cntl::cntl *drivecontroller;
	cntl::cntl *mechanismcontroller;
};

#endif /* SRC_CONTROLLERINPUT_CONTROLLERMANAGER_H_ */
