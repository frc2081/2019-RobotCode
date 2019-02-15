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
#include "LiftPIDControl.h"

class ClimbManager {

    public:
    ClimbManager(IO *io, RobotCommands *cmds);

    IO *_io;
    RobotCommands *_cmds;
    LiftPIDControl *lift;

    void ClimbManagerInit();
    void ClimbManagerTeleopPeriodic();
    void ClimbManagerRobotPeriodic();
    private:
    //TODO: get actual encoder values; this is incorrect
    double moveForwardStage1EncoderValue = 1000;
    double moveForwardStage2EncoderValue = 1000;

    //Climber movement Cals
    //Durations in milliseconds to move forward to get drivetrain front wheels over platform
    //Calcuation:
    //At full power, seat motor is 2.75 seconds per revolution
    //2.375" diameter wheel, 3.14*2.375 = 7.45 inches / rev
    //Full power motor speed = 7.45 inches per rev / 2.75 seconds per rev = 2.71 inches/sec
    
    //Desired first stage robot movement is 11.5 inches
    //11.5 inches / 2.71 inches /sec = 4.24 seconds = 4240 millseconds
    int moveForwardStage1Duration = 4240; 
    //Desired second stage robot movement is 13.25 inches
    //13.25 inches / 2.71 inches per sec = 4.89 seconds = 4890 milliseconds
    int moveForwardStage2Duration = 4890;
    int timer;

    double initialLiftDriveEncoderValue = 0;
    //TODO: get actual drive values; this is incorrect
    double drivetrainPower = .2;
    double liftMotorPower = 1.0;

    //state machine initialization
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
    int climbLevel = 0;

};