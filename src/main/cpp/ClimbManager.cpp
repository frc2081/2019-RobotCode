/*
 * ClimbManager.cpp
 *
 *  Created on: Jan 20, 2019
 *      Author: blzzrd
 */

#include "ClimbManager.h"

ClimbManager::ClimbManager() {

    ClimbManagerOutput = new ClimbManagerOutputs();

    state = STATE::robotOnFirstLevel;

}

ClimbManager::ClimbManagerInit() {
    initialLBLiftEncoderValue = liftlbenc;
    initialRBLiftEncoderValue = liftrbenc;
    initialLFLiftEncoderValue = liftlfenc;
    initialRFLiftEncoderValue = liftrfenc;
    initialLiftDriveEncoderValue = liftdriveenc;
}

ClimbManager::ClimbManagerPeriodic() {
    switch(state) {
        case STATE::robotOnFirstLevel:
            /*
            TODO:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           //liftlbmot, liftrbmot, liftlfmot, liftrfmot need to be turned off 

           if (climbCommand) {
               state = STATE::prepareToClimb;
           }
           break;
        case STATE::prepareToClimb:
            /*
            TODO:
            front extension motor: extending
            rack motor: stopped
            back extension motor: extending
            drivetrain: stopped
            */
           //liftlbmot, liftrbmot, liftlfmot, liftrfmot need to be turned on with specified power 

           if (climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           }
            else if (liftlbenc - initialLBLiftEncoderValue >= liftLBEncoderValue &&
                    liftrbenc - initialRBLiftEncoderValue >= liftRBEncoderValue &&
                    liftlfenc - initialLFLiftEncoderValue >= liftLFEncoderValue &&
                    liftrfenc - initialRFLiftEncoderValue >= liftRFEncoderValue            
            ) {
               state = STATE::moveForwardStage1;
               break;
           }
           break;
        case STATE::moveForwardStage1:
            /*
            TODO:
            front extension motor: stopped
            rack motor: moving forward
            back extension mottor: stopped
            drivetrain: stopped
            */
           //liftlbmot, liftrbmot, liftlfmot, liftrfmot need to be turned off 
           //liftdrivemot needs to be turned on

           if (climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           }
           else if (liftdriveenc - initialLiftDriveEncoderValue >= moveForwardState1EncoderValue)
           {
               state = STATE::robotStoppedHalfway;
               break;
           }
           break;
        case STATE::robotStoppedHalfway:
            /*
            TODO:
            front extension motor: retracting
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
           //liftlbmot, liftrbmot need to be turned off 
           //liftrfmot, liftlfmot need to be turned on in reverse
           //liftdrivemot needs to be turned on

           if (climbAbort) {
               state = STATE::robotClimbComplete;
               break;
           }
           else if (liftlfenc <= initialLFLiftEncoderValue &&
                    liftrfenc <= initialLFLiftEncoderValue)
           {
               state = STATE::moveForwardStage2;
               break;
           }
            break;
        case STATE::moveForwardStage2:
            /*
            TODO:
            front extension motor: stopped
            rack motor: moving forward
            back extension motor: stopped
            drivetrain: moving forward
            */
           //liftlbmot, liftrbmot, liftlfmot, liftrfmot need to be turned off 
           //liftdrivemot needs to be turned on
           //swerve drive motors need to be turned on


        case STATE::robotStoppedOnPlatform:
            /*
            TODO:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: retracting
            drivetrain: stopped
            */
        case STATE::robotClimbComplete:
            /*
            TODO:
            front extension motor: stopped
            rack motor: stopped
            back extension motor: stopped
            drivetrain: stopped
            */
    }
}