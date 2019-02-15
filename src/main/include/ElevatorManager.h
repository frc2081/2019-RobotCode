/*
 * ElevatorManager.h
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "IO.h"
#include "RobotCommands.h"

class ElevatorManager {


        public:

    enum class ElevatorManagerState{
        Transit,
        HatchPickupWait,
        HatchPickup,
        HatchPlaceL1Wait,
        HatchPlaceL1,
        HatchPlaceL2Wait,
        HatchPlaceL2,
        BallPickup,
        BallGrabbed,
        BallPlaceCargoWait,
        BallPlaceCargo,
        BallPlaceL1Wait,
        BallPlaceL1,
        BallPlaceL2Wait,
        BallPlaceL2,
        BallEject
    };

    ElevatorManagerState ElevatorManagerCurrentState;

    double ElevatorPosCmd;
    double BallIntakePowerCmd;
    double BallIntakeSpeedCmd;
    bool HatchClawPos;
    bool HatchArmPos;
    bool BallArmPos;
    bool ManualMode;
    bool frameStandPos;

    //temp values
    double ElevHomePos;
    double ElevHatchL1Pos;
    double ElevHatchL2Pos;
    double ElevBallPickupPos;
    double ElevBallCargoPos;
    double ElevBallL1Pos;
    double ElevBallL2Pos;
    double BallArmIntake;
    double BallArmIdle;
    double BallArmEject;
    double BallArmHold;
    bool extended;
    bool retracted;
    int BallEjectTimer;
    int BallEjectTimerLimit;
    int BallIntakeTimer;

    double intakeP = .5;
    double intakeI = 0;
    double intakeD = 0;
    double intakeF = .00833; //Motor slowest speed is 120 RPM, 1/120 = .008333

    PIDController *ballIntakePID;

    ElevatorManager(IO *io, RobotCommands *cmds);
    void ElevatorManagerPeriodic();
    void ElevatorManagerMechanism(IO *io);

    IO *_io;
    RobotCommands *_cmds;
};