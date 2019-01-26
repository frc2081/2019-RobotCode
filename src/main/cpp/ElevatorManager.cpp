/*
 * ElevatorManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "ElevatorManager.h"

ElevatorManager::ElevatorManager() {

    switch(state){
        case STATE::Transit:
            ElevatorPosCmd = ElevHomePos;
            HatchArmPos = extended;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::HatchPickupWait:
            ElevatorPosCmd = ElevHatchL1Pos;
            HatchArmPos = retracted;
            HatchClawPos = retracted;
            BallIntakePowerCmd = BallArmIdle;

            break;
        case STATE::HatchPickup:
            ElevatorPosCmd = ElevHatchL1Pos;
            HatchArmPos = retracted;
            HatchClawPos = extended;
            BallIntakePowerCmd = BallArmIdle;

            break;
        case STATE::HatchPlaceL1Wait:
            ElevatorPosCmd = ElevHatchL1Pos;
            HatchArmPos = retracted;
            HatchClawPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::HatchPlaceL1:
            ElevatorPosCmd = ElevHatchL1Pos;
            HatchArmPos = retracted;
            HatchClawPos = retracted;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::HatchPlaceL2Wait:
            ElevatorPosCmd = ElevHatchL2Pos;
            HatchArmPos = retracted;
            HatchClawPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::HatchPlaceL2:
            ElevatorPosCmd = ElevHatchL2Pos;
            HatchArmPos = retracted;
            HatchClawPos = retracted;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::BallPickup:
            ElevatorPosCmd = ElevBallPickupPos;
            HatchArmPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIntake;
            BallArmPos = retracted;

            break;
        case STATE::BallGrabbed:
            ElevatorPosCmd = ElevBallPickupPos;
            HatchArmPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle; //??
            BallArmPos = extended;

            break;
        case STATE::BallPlaceCargoWait:
            ElevatorPosCmd = ElevBallCargoPos;
            HatchArmPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = retracted;

            break;
        case STATE::BallPlaceCargo:
            ElevatorPosCmd = ElevBallCargoPos;
            HatchArmPos = extended;
            BallShooterPos = extended;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = retracted;

            break;
        case STATE::BallPlaceL1Wait:
            ElevatorPosCmd = ElevBallL1Pos;
            HatchArmPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = extended;

            break;
        case STATE::BallPlaceL1:
            ElevatorPosCmd = ElevBallL1Pos;
            HatchArmPos = extended;
            BallShooterPos = extended;
            BallIntakePowerCmd = BallArmEject;
            BallArmPos = extended;

            break;
        case STATE::BallPlaceL2Wait:
            ElevatorPosCmd = ElevBallL2Pos;
            HatchArmPos = extended;
            BallShooterPos = retracted;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = retracted;

            break;
        case STATE::BallPlaceL2:
            ElevatorPosCmd = ElevBall;
            HatchArmPos = extended;
            BallShooterPos = extended;
            BallIntakePowerCmd = BallArmIdle;
            BallArmPos = retracted;

            break;
        case STATE::BallEject:
            //if statement?
            BallShooterPos = extended;
            BallIntakePowerCmd = BallArmEject;

            break;
    }
    
}