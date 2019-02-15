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
    ElevBallPickupPos = 4;
    ElevBallCargoPos = 24;
    ElevBallL1Pos = 6;
    ElevBallL2Pos = 27;
    extended = true;
    retracted = false;
    BallEjectTimer = 0;
    BallEjectTimerLimit = 25;
    ManualMode = false;
    BallIntakeTimer = 0;

    //Ball intake motor speeds in RPM
    BallArmIntakeSpeed = -300;
    BallArmIdleSpeed = 0;
    BallArmEjectSpeed = 600;
    BallArmHoldSpeed = -120;

    ElevatorPosCmd = ElevHomePos;
    BallIntakeSpeedCmd = BallArmIdleSpeed;
    HatchClawPos = retracted;
    HatchArmPos = extended;
    BallArmPos = extended;
    frameStandPos = retracted;

    ballIntakePID = new frc::PIDController(intakeP, intakeI, intakeD, intakeF, _io->ballintakeenc, _io->ballintakemot, 0.05);
    ballIntakePID->Enable();

    frc::SmartDashboard::PutNumber("Intake Motor PID P", intakeP);
    frc::SmartDashboard::PutNumber("Intake Motor PID I", intakeI);
    frc::SmartDashboard::PutNumber("Intake Motor PID D", intakeD);
    frc::SmartDashboard::PutNumber("Intake Motor PID F", intakeF);
    }
    
void ElevatorManager::ElevatorManagerPeriodic(){

    intakeP = frc::SmartDashboard::GetNumber("Intake Motor PID P", intakeP);
    intakeI = frc::SmartDashboard::GetNumber("Intake Motor PID P", intakeI);
    intakeD = frc::SmartDashboard::GetNumber("Intake Motor PID P", intakeD);
    intakeF = frc::SmartDashboard::GetNumber("Intake Motor PID P", intakeF);

    ballIntakePID->SetP(intakeP);
    ballIntakePID->SetI(intakeI);
    ballIntakePID->SetD(intakeD);
    ballIntakePID->SetF(intakeF);

    //*********Main State Machine**********
    //If manual mode is active, bypass the state machine and control outputs directly
    //Otherwise let the state machine set the output values
    if(_cmds->manualModeActive){

        if(_cmds->hatchArmToggleManual) {HatchArmPos = !HatchArmPos;}
        if(_cmds->hatchClawManual) {HatchClawPos = !HatchClawPos;}
        if(_cmds->ballArmToggleManual) {BallArmPos = !BallArmPos;}
        if(_cmds->frameStandManual) {frameStandPos = !frameStandPos;}
        if(_cmds->ballArmMotorIntakeManual) {BallIntakeSpeedCmd = BallArmIntakeSpeed;}
        else {BallIntakeSpeedCmd = BallArmIdleSpeed;}

    } else {
        switch(ElevatorManagerCurrentState){
            case ElevatorManagerState::Transit: //0
                ElevatorPosCmd = ElevHomePos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->hatchPickup){ElevatorManagerCurrentState = ElevatorManagerState::HatchPickupWait;}
                if(_cmds->cargoPickup){ElevatorManagerCurrentState = ElevatorManagerState::BallPickup;}
                if(_cmds->placeHatchOne){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL1Wait;}
                if(_cmds->placeHatchTwo){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL2Wait;}
                if(_cmds->placeCargoInShip){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceCargoWait;}
                if(_cmds->placeCargoRocketOne){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL1Wait;}
                if(_cmds->placeCargoRocketTwo){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL2Wait;}

                break;
            case ElevatorManagerState::HatchPickupWait: //1
                ElevatorPosCmd = ElevHatchL1Pos;
                HatchArmPos = retracted;
                HatchClawPos = retracted;
                BallIntakeSpeedCmd = BallArmIdleSpeed;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->hatchPickup){ElevatorManagerCurrentState = ElevatorManagerState::HatchPickup;}
                if(_io->hatchDetectorOne->Get() && _io->hatchDetectorTwo->Get()) {ElevatorManagerCurrentState = ElevatorManagerState::HatchPickup;}

                break;
            case ElevatorManagerState::HatchPickup: //2
                ElevatorPosCmd = ElevHatchL1Pos;
                HatchArmPos = retracted;
                HatchClawPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::HatchPlaceL1Wait: //3
                ElevatorPosCmd = ElevHatchL1Pos;
                HatchArmPos = retracted;
                HatchClawPos = extended;

                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->placeHatchOne){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL1;}

                break;
            case ElevatorManagerState::HatchPlaceL1: //4
                ElevatorPosCmd = ElevHatchL1Pos;
                HatchArmPos = retracted;
                HatchClawPos = retracted;

                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::HatchPlaceL2Wait: //5
                ElevatorPosCmd = ElevHatchL2Pos;
                HatchArmPos = retracted;
                HatchClawPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->placeHatchTwo){ElevatorManagerCurrentState = ElevatorManagerState::HatchPlaceL2;}

                break;
            case ElevatorManagerState::HatchPlaceL2: //6
                ElevatorPosCmd = ElevHatchL2Pos;
                HatchArmPos = retracted;
                HatchClawPos = retracted;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::BallPickup: //7
                ElevatorPosCmd = ElevBallPickupPos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmIntakeSpeed;
                BallArmPos = retracted;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->cargoPickup){ElevatorManagerCurrentState = ElevatorManagerState::BallGrabbed;}

                break;
            case ElevatorManagerState::BallGrabbed: //8
                ElevatorPosCmd = ElevBallPickupPos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmHoldSpeed; //??
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::BallPlaceCargoWait: //9
                ElevatorPosCmd = ElevBallCargoPos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->placeCargoInShip){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceCargo;}

                break;
            case ElevatorManagerState::BallPlaceCargo: //10
                ElevatorPosCmd = ElevBallCargoPos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmEjectSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::BallPlaceL1Wait: //11
                ElevatorPosCmd = ElevBallL1Pos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->placeCargoRocketOne){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL1;}

                break;
            case ElevatorManagerState::BallPlaceL1: //12
                ElevatorPosCmd = ElevBallL1Pos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmEjectSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::BallPlaceL2Wait: //13
                ElevatorPosCmd = ElevBallL2Pos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmIdleSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                if(_cmds->placeCargoRocketTwo){ElevatorManagerCurrentState = ElevatorManagerState::BallPlaceL2;}

                break;
            case ElevatorManagerState::BallPlaceL2: //14
                ElevatorPosCmd = ElevBallL2Pos;
                HatchArmPos = extended;
                BallIntakeSpeedCmd = BallArmEjectSpeed;
                BallArmPos = extended;

                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}

                break;
            case ElevatorManagerState::BallEject: //15
                BallIntakeSpeedCmd = BallArmEjectSpeed;

                BallEjectTimer++;
                if(BallEjectTimer == BallEjectTimerLimit){
                    ElevatorManagerCurrentState = ElevatorManagerState::Transit;
                    BallEjectTimerLimit = 0;
                    }
                if(_cmds->elevatorHome){ElevatorManagerCurrentState = ElevatorManagerState::Transit;}
                break;
        }
    }

    if(_cmds->ejectCargo){ElevatorManagerCurrentState = ElevatorManagerState::BallEject;}

    frc::SmartDashboard::PutNumber("Elevator Current State: ", static_cast<double>(ElevatorManagerCurrentState));
    frc::SmartDashboard::PutBoolean("Elevator Hatch Arm Position", HatchArmPos);
    frc::SmartDashboard::PutBoolean("Elevator Hatch Claw Position", HatchClawPos);
    frc::SmartDashboard::PutBoolean("Elevator Ball Arm State:", BallArmPos);
    frc::SmartDashboard::PutNumber("Elevator Ball Intake Power", _io->ballintakemot->Get());
    frc::SmartDashboard::PutNumber("Elevator Ball Intake Setpoint", BallIntakeSpeedCmd);
    frc::SmartDashboard::PutBoolean("Elevator Hatch Detect 1", _io->hatchDetectorOne->Get());
    frc::SmartDashboard::PutBoolean("Elevator Hatch Detect 2", _io->hatchDetectorTwo->Get());
    frc::SmartDashboard::PutNumber("Elevator Motor Current", _io->PDP->GetCurrent(13));

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

    if(frameStandPos){
    io->framestandsolenoidout->Set(true);
    io->framestandsolenoidin->Set(false);
    }
    else{
    io->framestandsolenoidout->Set(false);
    io->framestandsolenoidin->Set(true);
    }

    if(!BallArmPos){
    io->ballarmsolenoidout->Set(true);
    io->ballarmsolenoidin->Set(false);
    }
    else{
    io->ballarmsolenoidout->Set(false);
    io->ballarmsolenoidin->Set(true);
    }

    ballIntakePID->SetSetpoint(BallIntakeSpeedCmd);
    _io->elevatorDesiredPos = ElevatorPosCmd;
}

