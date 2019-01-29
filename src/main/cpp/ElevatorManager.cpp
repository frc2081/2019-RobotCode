/*
 * ElevatorManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "ElevatorManager.h"

ElevatorManager::ElevatorManager(IO *io, RobotCommands *cmds) {

    _io = io;
    _cmds = cmds;

    ElevatorManagerCurrentState = ElevatorManagerState::Transit;

    ElevHomePos = 0;
    ElevHatchL1Pos = 15;
    ElevHatchL2Pos = 60;
    ElevBallPickupPos = 20;
    ElevBallCargoPos = 50;
    ElevBallL1Pos = 30;
    ElevBallL2Pos = 75;
    BallArmIntake = 1;
    BallArmIdle = 0;
    BallArmEject = -1;
    extended = true;
    retracted = false;

    ElevatorPosCmd = ElevHomePos;
    BallIntakePowerCmd = BallArmIdle;
    HatchClawPos = retracted;
    HatchArmPos = extended;
    BallShooterPos = retracted;
    BallArmPos = extended;

    }
    
void ElevatorManager::ElevatorManagerPeriodic(){

switch(ElevatorManagerCurrentState){
    case ElevatorManagerState::Transit:
        ElevatorPosCmd = ElevHomePos;
        HatchArmPos = extended;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::HatchPickupWait:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallIntakePowerCmd = BallArmIdle;

        break;
    case ElevatorManagerState::HatchPickup:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallIntakePowerCmd = BallArmIdle;

        break;
    case ElevatorManagerState::HatchPlaceL1Wait:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::HatchPlaceL1:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::HatchPlaceL2Wait:
        ElevatorPosCmd = ElevHatchL2Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::HatchPlaceL2:
        ElevatorPosCmd = ElevHatchL2Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::BallPickup:
        ElevatorPosCmd = ElevBallPickupPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIntake;
        BallArmPos = retracted;

        break;
    case ElevatorManagerState::BallGrabbed:
        ElevatorPosCmd = ElevBallPickupPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle; //??
        BallArmPos = extended;

        break;
    case ElevatorManagerState::BallPlaceCargoWait:
        ElevatorPosCmd = ElevBallCargoPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        break;
    case ElevatorManagerState::BallPlaceCargo:
        ElevatorPosCmd = ElevBallCargoPos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        break;
    case ElevatorManagerState::BallPlaceL1Wait:
        ElevatorPosCmd = ElevBallL1Pos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::BallPlaceL1:
        ElevatorPosCmd = ElevBallL1Pos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmEject;
        BallArmPos = extended;

        break;
    case ElevatorManagerState::BallPlaceL2Wait:
        ElevatorPosCmd = ElevBallL2Pos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        break;
    case ElevatorManagerState::BallPlaceL2:
        ElevatorPosCmd = ElevBallL2Pos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        break;
    case ElevatorManagerState::BallEject:
        //if statement?
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmEject;

        break;

    }

}

