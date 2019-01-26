/*
 * ElevatorManager.h
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "IO.h"
#include "RobotCommands.h"

class ElevatorManager {

    enum class STATE{
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
    }

    double ElevatorPosCmd;
    double BallIntakePowerCmd;
    bool HatchClawPos;
    bool HatchArmPos;
    bool BallShooterPos;
    bool BallArmPos;

    //temp values
    double ElevHomePos = 0;
    double ElevHatchL1Pos = 15;
    double ElevHatchL2Pos = 60;
    double ElevBallPickupPos = 20;
    double ElevBallCargoPos = 50;
    double ElevBallL1Pos = 30;
    double ElevBallL2Pos = 75;
    double BallArmIntake = 1;
    double BallArmIdle = 0;
    double BallArmEject = -1;
    bool extended = true;
    bool retracted = false;



    public:
    ElevatorManager();
};