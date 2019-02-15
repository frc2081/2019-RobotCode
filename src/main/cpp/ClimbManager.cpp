/*
 * ClimbManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 *      Contributor: IChism
 */

#include "ClimbManager.h"

ClimbManager::ClimbManager(IO *io, RobotCommands *cmds) {
    _io = io;
    _cmds = cmds;
    lift = new LiftPIDControl(io, cmds);
    timer = 0;

    climbState = ClimbSTATE::robotOnFirstLevel;
}

void ClimbManager::ClimbManagerInit() {
    initialLiftDriveEncoderValue = _io->liftdriveenc->Get();
}
void ClimbManager::ClimbManagerRobotPeriodic() {
    lift->liftPIDControlRobotPeriodic();
}

void ClimbManager::ClimbManagerTeleopPeriodic() {
    lift->liftPIDControlTeleopPeriodic();
    //display current values on the Smart Dashboard
    frc::SmartDashboard::PutNumber("ClimbManager current State", static_cast<int>(climbState));
    frc::SmartDashboard::PutNumber("Desired FRONT lift position", static_cast<int>(lift->liftFrontPosDes));
    frc::SmartDashboard::PutNumber("Desired REAR lift position", static_cast<int>(lift->liftRearPosDes));
    frc::SmartDashboard::PutNumber("Actual FRONT lift position", static_cast<int>(lift->liftFrontPosAct));
    frc::SmartDashboard::PutNumber("Actual REAR lift position", static_cast<int>(lift->liftRearPosAct));
    frc::SmartDashboard::PutNumber("Lift Drive Motor Encoder", _io->liftdriveenc->Get());
    frc::SmartDashboard::PutBoolean("Fast Climber", lift->moveFast);
    frc::SmartDashboard::PutBoolean("Sync Front and Rear Lifts", lift->syncFrontRearLifts);
    frc::SmartDashboard::PutBoolean("Abort Climb Command is Active", _cmds->climbAbort);
    frc::SmartDashboard::PutBoolean("Start Climb lvl2 Command is Active", _cmds->climbCommandLevelTwo);
    frc::SmartDashboard::PutBoolean("Start Climb lvl1 Command is Active", _cmds->climbCommandLevelOne);
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

           timer = 0;
           lift->liftFrontPosDes = lift->liftPos::RETRACTED;
           lift->liftRearPosDes = lift->liftPos::RETRACTED;

           if (_cmds->climbCommandLevelTwo) {
               climbLevel = 2;
               climbState = ClimbSTATE::prepareToClimb;
           } else if (_cmds->climbCommandLevelOne) {
               climbLevel = 1;
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
            if (climbLevel == 2) {
                lift->liftFrontPosDes = lift->liftPos::EXTENDEDLEVELTWO;
                lift->liftRearPosDes = lift->liftPos::EXTENDEDLEVELTWO;
            } else if (climbLevel == 1) {
                lift->liftFrontPosDes = lift->liftPos::EXTENDEDLEVELONE;
                lift->liftRearPosDes = lift->liftPos::EXTENDEDLEVELONE;
            }
            lift->moveFast = false;
            lift->syncFrontRearLifts = true;

            //Take drivetrain control away from the driver
            _cmds->guidanceSysActive = true;
            _cmds->autodrvang = 0;
            _cmds->autodrvmag = 0;
            _cmds->autodrvrot = 0;

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           } else if (lift->liftFrontPosDes == lift->liftFrontPosAct &&
                    lift->liftRearPosDes == lift->liftRearPosAct) {
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

           //Add 20ms to climb timer
           timer += 20;

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           }  else if (timer >= moveForwardStage1Duration) {
              climbState = ClimbSTATE::robotStoppedHalfway;
              timer = 0;
              break;
           /*else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage1EncoderValue) {
               climbState = ClimbSTATE::robotStoppedHalfway;
               break;*/
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
           lift->liftFrontPosDes = lift->liftPos::RETRACTED;
           lift->moveFast = true;
           lift->syncFrontRearLifts = false;     
           //liftdrivemot needs to be turned off
           _io->liftdrivemot->Set(0);

           if (_cmds->climbAbort) {
               climbState = ClimbSTATE::robotClimbComplete;
               break;
           } else if (lift->liftFrontPosDes == lift->liftFrontPosAct) {
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
            _cmds->guidanceSysActive = true;
            _cmds->autodrvang = 0;
            _cmds->autodrvmag = drivetrainPowerLow;
            _cmds->autodrvrot = 0;
            
            //Add 20ms to climb timer
            timer += 20;

           if (_cmds->climbAbort) {
                climbState = ClimbSTATE::robotClimbComplete;
                break;
           } else if (timer >= moveForwardStage2Duration) {
              climbState = ClimbSTATE::robotStoppedOnPlatform;
              timer = 0;
              break;
               /*else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage2EncoderValue) {
                climbState = ClimbSTATE::robotStoppedOnPlatform;
                break; */
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
           timer = 0;
           lift->liftRearPosDes = lift->liftPos::RETRACTED;

           //liftdrivemot needs to be turned off
           _io->liftdrivemot->Set(0);
           //swerve motors need to be off
            _cmds->guidanceSysActive = true;
            _cmds->autodrvang = 0;
            _cmds->autodrvmag = drivetrainPowerHold;
            _cmds->autodrvrot = 0;

           if (_cmds->climbAbort ||
                lift->liftRearPosDes == lift->liftRearPosAct) {
                climbState = ClimbSTATE::moveForwardStage3;
                break;
           }
            break;
        case ClimbSTATE::moveForwardStage3:
            /*
            STATE FEATURES:
            front extension motor: stopped
            rack motor: moving forward
            back extension motor: stopped
            drivetrain: moving forward
            */
           //liftdrivemot stopped
           _io->liftdrivemot->Set(0);

           //swerve drive motors need to be turned on
            _cmds->guidanceSysActive = true;
            _cmds->autodrvang = 0;
            _cmds->autodrvmag = drivetrainPowerPullForward;
            _cmds->autodrvrot = 0;
            
            //Add 20ms to climb timer
            timer += 20;

           if (_cmds->climbAbort) {
                climbState = ClimbSTATE::robotClimbComplete;
                break;
           } else if (timer >= moveForwardStage3Duration) {
              climbState = ClimbSTATE::robotClimbComplete;
              timer = 0;
              break;
               /*else if (_io->liftdriveenc->Get() - initialLiftDriveEncoderValue >=
            moveForwardStage2EncoderValue) {
                climbState = ClimbSTATE::robotStoppedOnPlatform;
                break; */
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
           lift->liftFrontPosDes = lift->liftPos::RETRACTED;
           lift->liftRearPosDes = lift->liftPos::RETRACTED;
           lift->syncFrontRearLifts = true;     
           lift->moveFast = false;

           timer = 0;

            //Return drivetrain control to the driver
            _cmds->guidanceSysActive = false;
            _cmds->autodrvang = 0;
            _cmds->autodrvmag = 0;
            _cmds->autodrvrot = 0;
           _io->liftdrivemot->Set(0);
           //swerve motors need to be off'
            climbState = ClimbSTATE::robotOnFirstLevel;
           break;
    }
}