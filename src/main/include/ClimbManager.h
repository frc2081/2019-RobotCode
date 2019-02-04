/*
 * ClimbManager.h
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 *      Contributor: IChism
 */

#include "RobotCommands.h"
#include "IO.h"
#include "frc/WPIlib.h"
#include "liftPIDControl.h"

class ClimbManager {

    public:
    ClimbManager(IO *io, RobotCommands *cmds);

    IO *_io;
    RobotCommands *_cmds;
    LiftPIDControl *lift;

    void ClimbManagerInit();
    void ClimbManagerPeriodic();
    private:
    //TODO: get actual encoder values; this is incorrect
    double moveForwardStage1EncoderValue = 1000;
    double moveForwardStage2EncoderValue = 1000;

    double initialLiftDriveEncoderValue = 0;\
    //TODO: get actual drive values; this is incorrect
    double drivetrainPower = 0.05;
    double liftMotorPower = 0.05;

    enum class ClimbSTATE {
        robotOnFirstLevel,
        prepareToClimb,
        moveForwardStage1,
        robotStoppedHalfway,
        moveForwardStage2,
        robotStoppedOnPlatform,
        robotClimbComplete,
    };

    ClimbSTATE climbState;
};