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
    ElevHatchL1Pos = 4;
    ElevHatchL2Pos = 27;
    ElevBallPickupPos = 20;
    ElevBallCargoPos = 24;
    ElevBallL1Pos = 6;
    ElevBallL2Pos = 27;
    BallArmIntake = 1;
    BallArmIdle = 0;
    BallArmEject = -1;
    extended = true;
    retracted = false;
    BallEjectTimer = 0;
    BallEjectTimerLimit = 25;

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

        if(_cmds->hatchPickup){ElevatorManagerCurrentState = ElevatorManagerState::HatchPickupWait;}
        if(_cmds->cargoPickup){ElevatorManagerCurrentState = ElevatorManagerState::BallPickup;}
        if(_cmds->placeHatchOne){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL1Wait;}
        if(_cmds->placeHatchTwo){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL2Wait;}
        if(_cmds->placeCargoInShip){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceCargoWait;}
        if(_cmds->placeCargoRocketOne){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL1Wait;}
        if(_cmds->placeCargoRocketTwo){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL2Wait;}

        break;
    case ElevatorManagerState::HatchPickupWait:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallIntakePowerCmd = BallArmIdle;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->hatchPickup){ElevatorManagerCurrentState = ElevatorManagerState::HatchPickup;}
        if(_io->hatchDetectorOne->Get() && _io->hatchDetectorTwo()) {ElevatorManagerCurrentState = ElevatorManagerState::HatchPickup;}

        break;
    case ElevatorManagerState::HatchPickup:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallIntakePowerCmd = BallArmIdle;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::HatchPlaceL1Wait:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->placeHatchOne){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL1;}

        break;
    case ElevatorManagerState::HatchPlaceL1:
        ElevatorPosCmd = ElevHatchL1Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::HatchPlaceL2Wait:
        ElevatorPosCmd = ElevHatchL2Pos;
        HatchArmPos = retracted;
        HatchClawPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->placeHatchTwo){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL2;}

        break;
    case ElevatorManagerState::HatchPlaceL2:
        ElevatorPosCmd = ElevHatchL2Pos;
        HatchArmPos = retracted;
        HatchClawPos = retracted;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::BallPickup:
        ElevatorPosCmd = ElevBallPickupPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIntake;
        BallArmPos = retracted;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->cargoPickup){ElevatorManagerCurrentState = ElevatorManagerState::BallGrabbed;}

        break;
    case ElevatorManagerState::BallGrabbed:
        ElevatorPosCmd = ElevBallPickupPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle; //??
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::BallPlaceCargoWait:
        ElevatorPosCmd = ElevBallCargoPos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->placeCargoInShip){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceCargo;}

        break;
    case ElevatorManagerState::BallPlaceCargo:
        ElevatorPosCmd = ElevBallCargoPos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::BallPlaceL1Wait:
        ElevatorPosCmd = ElevBallL1Pos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->placeCargoRocketOne){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL1;}

        break;
    case ElevatorManagerState::BallPlaceL1:
        ElevatorPosCmd = ElevBallL1Pos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmEject;
        BallArmPos = extended;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::BallPlaceL2Wait:
        ElevatorPosCmd = ElevBallL2Pos;
        HatchArmPos = extended;
        BallShooterPos = retracted;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        if(_cmds->placeCargoRocketTwo){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL2;}

        break;
    case ElevatorManagerState::BallPlaceL2:
        ElevatorPosCmd = ElevBallL2Pos;
        HatchArmPos = extended;
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmIdle;
        BallArmPos = retracted;

        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

        break;
    case ElevatorManagerState::BallEject:
        //if statement?
        BallShooterPos = extended;
        BallIntakePowerCmd = BallArmEject;

        BallEjectTimer++;
        if(BallEjectTimer == BallEjectTimerLimit){
            ElevatorManagerCurrentState = ElevatorManagerState::Transit;
            BallEjectTimerLimit = 0;
            }
        if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
        break;

    }

    if(_cmds->ejectCargo){ElevatorManagerCurrentState = ElevatorManagerState::BallEject;}

    if(_cmds->hatchArmToggleManual) {HatchArmPos = !HatchArmPos;}
    if(_cmds->hatchClawManual) {HatchClawPos = !HatchClawPos;}
    if(_cmds->ballArmToggleManual) {BallArmPos = !BallArmPos;}
    if(_cmds->ballEjectorManual) {BallShooterPos = !BallShooterPos;}
    //TODO: Add Robot CG shifter pnuematics

    frc::SmartDashboard::PutNumber("Elevator Current State: ", static_cast<double>(ElevatorManagerCurrentState));
    frc::SmartDashboard::PutBoolean("Elevator Hatch Arm Position", HatchArmPos);
    frc::SmartDashboard::PutBoolean("Elevator Hatch Claw Position", HatchClawPos);
    _io->elevatorDesiredPos = ElevatorPosCmd;
}

void ElevatorManager::ElevatorManagerMechanism(IO *io){
    if(HatchArmPos){
    io->hatcharmsolenoidin->Set(true);
    io->hatcharmsolenoidout->Set(false);
    }
    else{
    io->hatcharmsolenoidin->Set(false);
    io->hatcharmsolenoidout->Set(true);
    }

    if(HatchClawPos){
    io->hatchclawsolenoidout->Set(true);
    io->hatchclawsolenoidin->Set(false);
    }
    else{
    io->hatchclawsolenoidout->Set(false);
    io->hatchclawsolenoidin->Set(true);
    }

    if(BallShooterPos){
    io->ballshootersolenoidout->Set(true);
    io->ballshootersolenoidin->Set(false);
    }
    else{
    io->ballshootersolenoidout->Set(false);
    io->ballshootersolenoidin->Set(true);
    }

    if(BallArmPos){
    io->ballarmsolenoidout->Set(true);
    io->ballarmsolenoidin->Set(false);
    }
    else{
    io->ballarmsolenoidout->Set(false);
    io->ballarmsolenoidin->Set(true);
    }
}

