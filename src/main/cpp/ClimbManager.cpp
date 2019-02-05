/*
 * ClimbManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 *      Contributor: IChism
 */

#include "ClimbManager.h"

ClimbManager::ClimbManager(IO *io, RobotCommands *cmds, LiftPIDControl *cntrl) {

    _io = io;
    _cmds = cmds;
    _cntrl = cntrl;

    climbState = ClimbSTATE::robotOnFirstLevel;

}

void ClimbManager::ClimbManagerInit() {
    initialLiftDriveEncoderValue = _io->liftdriveenc->Get();
}

void ClimbManager::ClimbManagerPeriodic() {
    //display current values on the Smart Dashboard
    frc::SmartDashboard::PutNumber("ClimbManager current State", static_cast<int>(climbState));
    frc::SmartDashboard::PutNumber("Desired FRONT lift position", static_cast<int>(liftFrontPosDes));
    frc::SmartDashboard::PutNumber("Desired REAR lift position", static_cast<int>(liftRearPosDes));
    frc::SmartDashboard::PutNumber("Actual FRONT lift position", static_cast<int>(liftFrontPosAct));
    frc::SmartDashboard::PutNumber("Actual REAR lift position", static_cast<int>(liftRearPosAct));
    frc::SmartDashboard::PutNumber("Lift Drive Motor Encoder", _io->liftdriveenc->Get());
    frc::SmartDashboard::PutBoolean("Fast Climber", moveFast);
    frc::SmartDashboard::PutBoolean("Sync Front and Rear Lifts", syncFrontRearLifts);
    frc::SmartDashboard::PutBoolean("Abort Climb Command is Active", _cmds->climbAbort);
    frc::SmartDashboard::PutBoolean("Start Climb Command is Active", _cmds->climbCommand);
    //state machine
    switch(climbState) {
        case ClimbSTATE::robotOnFirstLevel:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           _cntrl->liftFrontPosDes = liftPos::RETRACTED;
           _cntrl->liftRearPosDes = liftPos::RETRACTED;
           _cmds->drvang = 0;
           _cmds->drvmag = 0;
           _cmds->drvrot = 0;

           if (_cmds->climbCommand) {
               climbState = ClimbSTATE::prepareToClimb;
           }
           break;
        case ClimbSTATE::prepareToClimb:
            /*
            STATE FEATURES:
            front extension motor: extending
            rack motor: stopped
            back extension motor: extending
            drivetrain: stopped
            */
           _cntrl->liftFrontPosDes = liftPos::EXTENDEDLEVELTWO;
           _cntrl->liftRearPosDes = liftPos::EXTENDEDLEVELTWO;
           _cntrl->moveFast = false;
           _cntrl->syncFrontRearLifts = true;

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           } else if (_cntrl->liftFrontPosDes == _cntrl->liftFrontPosAct && 
           _cntrl->liftRearPosDes == _cntrl->liftRearPosAct) {
               climbState = ClimbSTATE::moveForwardStage1;
               break;
           }
           break;
        case ClimbSTATE::moveForwardStage1:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: moving forward
            back extension mottor: stopped
            drivetrain: stopped
            */
           //liftdrivemot needs to be turned on
           _io->liftdrivemot->Set(liftMotorPower);

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           } else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage1EncoderValue) {
               climbState = ClimbSTATE::robotStoppedHalfway;
               break;
           }
           break;
        case ClimbSTATE::robotStoppedHalfway:
            /*
            STATE FEATURES:
            front extension motor: retracting
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           _cntrl->liftFrontPosDes = liftPos::RETRACTED;
           _cntrl->moveFast = true;
           _cntrl->syncFrontRearLifts = false;     
           //liftdrivemot needs to be turned off
           _io->liftdrivemot->Set(0);

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           } else if (_cntrl->liftFrontPosDes == _cntrl->liftFrontPosAct) {
               climbState = ClimbSTATE::moveForwardStage2;
               break;
           }
            break;
        case ClimbSTATE::moveForwardStage2:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: moving forward
            back extension motor: stopped
            drivetrain: moving forward
            */
           //liftdrivemot needs to be turned on
           _io->liftdrivemot->Set(liftMotorPower);

           //swerve drive motors need to be turned on
           _cmds->drvang = 0;
           _cmds->drvmag = drivetrainPower;
           _cmds->drvrot = 0;

           if (_cmds->climbAbort) {
                climbState = ClimbSTATE::robotClimbComplete;
                break;
           } else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage2EncoderValue) {
                climbState = ClimbSTATE::robotStoppedOnPlatform;
                break;
           }
            break;
        case ClimbSTATE::robotStoppedOnPlatform:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: retracting
            drivetrain: stopped
            */
           _cntrl->liftRearPosDes = liftPos::RETRACTED;

           //liftdrivemot needs to be turned off
           _io->liftdrivemot->Set(0);
           //swerve motors need to be off
           _cmds->drvmag = 0;

           if (_cmds->climbAbort ||
                _cntrl->liftRearPosDes == _cntrl->liftFrontPosAct) {
                climbState = ClimbSTATE::robotClimbComplete;
                break;
           }
            break;
        case ClimbSTATE::robotClimbComplete:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           _cntrl->liftFrontPosDes = liftPos::RETRACTED;
           _cntrl->liftRearPosDes = liftPos::RETRACTED;
           _cntrl->syncFrontRearLifts = true;     
           _cntrl->moveFast = false;
           _cmds->drvang = 0;
           _cmds->drvmag = 0;
           _cmds->drvrot = 0;
           _io->liftdrivemot->Set(0);
           //swerve motors need to be off
           break;
    }
}