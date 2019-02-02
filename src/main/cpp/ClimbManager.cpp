/*
 * ClimbManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "ClimbManager.h"

ClimbManager::ClimbManager(IO *io, RobotCommands *cmds) {

    _io = io;
    _cmds = cmds;

    state = STATE::robotOnFirstLevel;

}

void ClimbManager::ClimbManagerInit() {
    initialLiftDriveEncoderValue = _io->liftdriveenc->Get();
}

void ClimbManager::ClimbManagerPeriodic() {
    switch(state) {
        case STATE::robotOnFirstLevel:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           liftFrontPosDes = liftPos::RETRACTED;
           liftRearPosDes = liftPos::RETRACTED;
           _cmds->drvang = 0;
           _cmds->drvmag = 0;
           _cmds->drvrot = 0;

           if (_cmds->climbCommand) {
               state = STATE::prepareToClimb;
           }
           break;
        case STATE::prepareToClimb:
            /*
            STATE FEATURES:
            front extension motor: extending
            rack motor: stopped
            back extension motor: extending
            drivetrain: stopped
            */
           liftFrontPosDes = liftPos::EXTENDEDLEVELTWO;
           liftRearPosDes = liftPos::EXTENDEDLEVELTWO;
           moveFast = false;
           syncFrontRearLifts = true;

           if (_cmds->climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           } else if (liftFrontPosDes == liftFrontPosAct && liftRearPosDes == liftRearPosAct) {
               state = STATE::moveForwardStage1;
               break;
           }
           break;
        case STATE::moveForwardStage1:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: moving forward
            back extension mottor: stopped
            drivetrain: stopped
            */
           //liftdrivemot needs to be turned on
           

           if (_cmds->climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           } else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage1EncoderValue) {
               state = STATE::robotStoppedHalfway;
               break;
           }
           break;
        case STATE::robotStoppedHalfway:
            /*
            STATE FEATURES:
            front extension motor: retracting
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           liftFrontPosDes = liftPos::RETRACTED;
           moveFast = true;
           syncFrontRearLifts = false;     
           //liftdrivemot needs to be turned off

           if (_cmds->climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           } else if (liftFrontPosDes == liftFrontPosAct) {
               state = STATE::moveForwardStage2;
               break;
           }
            break;
        case STATE::moveForwardStage2:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: moving forward
            back extension motor: stopped
            drivetrain: moving forward
            */
           //liftdrivemot needs to be turned on
           //swerve drive motors need to be turned on
           _cmds->drvang = 0;
           _cmds->drvmag = 0.05;
           _cmds->drvrot = 0;

           if (_cmds->climbAbort) {
                state = STATE::robotClimbComplete;
                break;
           } else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage2EncoderValue) {
                state = STATE::robotStoppedOnPlatform;
                break;
           }
            break;
        case STATE::robotStoppedOnPlatform:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: retracting
            drivetrain: stopped
            */
           liftRearPosDes = liftPos::RETRACTED;

           //liftdrivemot needs to be turned off
           //swerve motors need to be off
           _cmds->drvmag = 0;

           if (_cmds->climbAbort ||
                liftRearPosDes == liftFrontPosAct) {
                state = STATE::robotClimbComplete;
                break;
           }
            break;
        case STATE::robotClimbComplete:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           liftFrontPosDes = liftPos::RETRACTED;
           liftRearPosDes = liftPos::RETRACTED;
           syncFrontRearLifts = true;     
           moveFast = false;
           _cmds->drvang = 0;
           _cmds->drvmag = 0;
           _cmds->drvrot = 0;

           //swerve motors need to be off
           break;
    }
}