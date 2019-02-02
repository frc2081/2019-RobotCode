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
    bool HatchClawPos;
    bool HatchArmPos;
    bool BallShooterPos;
    bool BallArmPos;

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
    bool extended;
    bool retracted;
    int BallEjectTimer;
    int BallEjectTimerLimit;

    ElevatorManager(IO *io, RobotCommands *cmds);
    void ElevatorManagerPeriodic();
    void ElevatorManagerMechanism(IO *io);

    IO *_io;
    RobotCommands *_cmds;
};