/*
 * ClimbManager.h
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "RobotCommands.h"
#include "IO.h"
//#include "liftPID.h"

class ClimbManager {

    public:
    ClimbManager(IO *io, RobotCommands *cmds);

    IO *_io;
    RobotCommands *_cmds;

    void ClimbManagerInit();
    void ClimbManagerPeriodic();
    private:
    //TODO: get actual encoder values; this is incorrect
    double moveForwardStage1EncoderValue = 1000;
    double moveForwardStage2EncoderValue = 1000;

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
    //TEMPORARY STUFF:: to be replaced when we get an actual source module for this
    enum class liftPos {
        RETRACTED,
        EXTENDEDLEVELONE,
        EXTENDEDLEVELTWO
    };

    liftPos liftFrontPosDes, liftFrontPosAct, liftRearPosDes, liftRearPosAct;
    bool moveFast, syncFrontRearLifts;
};