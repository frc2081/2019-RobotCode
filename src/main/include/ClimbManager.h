/*
 * ClimbManager.h
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "RobotCommands.h"
#include "IO.h"

class ClimbManager {

    public:
    ClimbManager();
    private:
    //TODO: get actual encoder values; this is incorrect
    double liftRBEncoderValue = 1000;
    double liftLBEncoderValue = 1000;
    double liftRFEncoderValue = 1000;
    double liftLFEncoderValue = 1000;
    double moveForwardState1EncoderValue = 1000;

    double initialLBLiftEncoderValue = 0;
    double initialRBLiftEncoderValue = 0;
    double initialLFLiftEncoderValue = 0;
    double initialRFLiftEncoderValue = 0;
    double initialLiftDriveEncoderValue = 0;

    enum class STATE {
        robotOnFirstLevel,
        prepareToClimb,
        moveForwardStage1,
        robotStoppedHalfway,
        moveForwardStage2,
        robotStoppedOnPlatform,
        robotClimbComplete,
    };

    STATE state;
};