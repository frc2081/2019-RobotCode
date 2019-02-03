#include "GuidanceSystem.h"

GuidanceSystem::GuidanceSystem(RobotCommands *cmds){
    _cmds = cmds;
    errorCenter = 0;
    errorAngle = 0;
    errorHeight = 0;
    drvMagCenter = 0;
    drvMagRange = 0;
    drvMagFinal = 0;
    drvAng = 0;
    drvRot = 0;
    timer = 0;

    GuidanceState = GuidanceSysState::NOTACTIVE;
}

void GuidanceSystem::GuidanceSystemPeriodic(){

    //Set the guidance system active if any auto command is active
    if(_cmds->autoHatchPickup || _cmds->autoPlaceHatchOne ||
        _cmds->autoPlaceHatchTwo ||  _cmds->autoPlaceCargoInShip ||
       _cmds->autoPlaceCargoRocketOne || _cmds->autoPlaceCargoRocketTwo) 
       {guidanceSysActive = true;}
    else {guidanceSysActive = false;}

    //Guidance system state machine
    switch(GuidanceState){
        case GuidanceSysState::NOTACTIVE:
            /*Determine if any command that uses the guidance system is present.
            If one is and the vision system has a target, set the matching elevator 
            command true for one loop to get the elevator system into placement position
            and then begin homing on the target*/

            if(targetAcquired == true && guidanceSysActive == true){
                GuidanceState = GuidanceSysState::TRACKING;
                //Must notify drivetrain control that guidance system is now in control of robot

                if(_cmds->autoHatchPickup == true) {_cmds->hatchPickup = true;}
                else if (_cmds->autoPlaceHatchOne == true) {_cmds->placeHatchOne = true; }
                else if(_cmds->autoPlaceHatchTwo == true) {_cmds->placeHatchTwo = true;}
                else if(_cmds->autoPlaceCargoInShip == true) {_cmds->placeCargoInShip = true;}
                else if(_cmds->autoPlaceCargoRocketOne == true) {_cmds->placeCargoRocketOne = true;}
                else if(_cmds->autoPlaceCargoRocketTwo == true) {_cmds->placeCargoRocketTwo = true;}
            }
            timer = 0;
            break;

        //Active homing on targets using vision system
        case GuidanceSysState::TRACKING:
            calcHomingVectors();
            //Set all elevator commands false to hold the elevator system in
            //place until the robot is ready to place the game piece.
            _cmds->hatchPickup = false;
            _cmds->placeHatchOne = false;
            _cmds->placeHatchTwo = false;
            _cmds->placeCargoInShip = false;
            _cmds->placeCargoRocketOne = false;
            _cmds->placeCargoRocketTwo = false;

            //Exit when robot has approached close enough to lose target tracking
            if(guidanceSysActive == false) {GuidanceState = GuidanceSysState::NOTACTIVE;}
            else if (targetAcquired == false) {GuidanceState = GuidanceSysState::FINALAPPROACH;}
            break;
        
        //Timed drive forward to close the last distance with the target after the vision system has lost tracking
        case GuidanceSysState::FINALAPPROACH:
            timer++;
            drvAng = finalApproachDrvAng;
            drvMagFinal = finalApproachDrvMag;
            drvRot = finalApproachDrvRot;
            
            if(guidanceSysActive == false) {GuidanceState = GuidanceSysState::NOTACTIVE;}
            else if (timer >= finalApproachDuration) {
                GuidanceState = GuidanceSysState::PLACEMENT;
                timer = 0;
            }
            break;  

        //Timed wait for game piece placement to happen
        case GuidanceSysState::PLACEMENT:
            timer++;

            //Stop drivetrain
            drvAng = 0;
            drvMagFinal = 0;
            drvRot = 0;

            //Issue command to the elevator system to place the game piece
            if(_cmds->autoHatchPickup == true) {_cmds->hatchPickup = true;}
            else if (_cmds->autoPlaceHatchOne == true) {_cmds->placeHatchOne = true; }
            else if(_cmds->autoPlaceHatchTwo == true) {_cmds->placeHatchTwo = true;}
            else if(_cmds->autoPlaceCargoInShip == true) {_cmds->placeCargoInShip = true;}
            else if(_cmds->autoPlaceCargoRocketOne == true) {_cmds->placeCargoRocketOne = true;}
            else if(_cmds->autoPlaceCargoRocketTwo == true) {_cmds->placeCargoRocketTwo = true;}
            
            if(guidanceSysActive == false) {GuidanceState = GuidanceSysState::NOTACTIVE;}
            else if (timer >= placementDuration) {
                GuidanceState = GuidanceSysState::BACKAWAY;
                timer = 0;
            }
            break;

        //Timed drive backwards to separate robot from target
        case GuidanceSysState::BACKAWAY:
            timer++;

            drvAng = finalApproachDrvAng;
            drvMagFinal = finalApproachDrvMag;
            drvRot = finalApproachDrvRot;

            _cmds->hatchPickup = false;
            _cmds->placeHatchOne = false;
            _cmds->placeHatchTwo = false;
            _cmds->placeCargoInShip = false;
            _cmds->placeCargoRocketOne = false;
            _cmds->placeCargoRocketTwo = false;

            if(guidanceSysActive == false) {GuidanceState = GuidanceSysState::NOTACTIVE;}
            else if (timer >= backAwayDuration) {
                GuidanceState = GuidanceSysState::DONE;
                timer = 0;
                _cmds->elevatorHome = true;
            }
            break;   

        //Delay state during which guidance cannot be restarted to avoid immediately 
        //reactivating the guidance system if the driver still has the button held down
        case GuidanceSysState::DONE:
            timer++;
            //Set guidanceSysActive false to override mechanism controller and give
            //drivetrain control back to the driver
            guidanceSysActive = false;

            if (timer >= guidanceCompleteDelay) {
                GuidanceState = GuidanceSysState::NOTACTIVE;
                timer = 0;
            }
            break;  

        default:
            GuidanceState = GuidanceSysState::NOTACTIVE;
            guidanceSysActive = false;
            break;  
   }

    //If guidance is active, tell the drivetrain and set the auto drive commands
    if(guidanceSysActive == true){
        _cmds->guidanceSysActive = true;
        _cmds->autodrvang = drvAng;
        _cmds->autodrvmag = drvMagFinal;
        _cmds->autodrvrot = drvRot;
    } else { _cmds->guidanceSysActive = false; }

    updateDashboard();

    //TODO:Add capability to execute each stage of homing independently
}

void GuidanceSystem::calcHomingVectors() {
    //If a target is present, calculate drive vectors to navigate robot to the target
    if(targetAcquired == true)
    {
        //Attempt to align the robot horizontally by centering the
        //targets in the camera image
        //Calculates drive angle as -180 to 180
        //Must be converted to 0 to 360 for swerve drive
        errorCenter = (distanceLeftTarget + distanceRightTarget);
        drvAng = errorCenter * kpCenterAng;
        drvMagCenter = errorCenter * kpCenterMag;

        //Attempt to align the facing of the robot by rotating until
        //the targets are the same width (0 parallax means we are square
        //to the target)
        //Note: kp is negative for correct rotation direction
        errorAngle = (widthLeftTarget - widthRightTarget);
        drvRot = errorAngle * kpAngle;

        //Drive the robot forward until the targets match a
        //predetermined desires height that puts the robot at the
        //correct distance from the target
        errorHeight = (targetDesiredHeight - heightRightTarget);
        drvMagRange = errorHeight * kpRange;

        //Select final drive mag to be larger of the two requested values
        //This ensures that the drv mag will always be non-zero until both
        //errors have reached 0
        if(drvMagRange >= drvMagCenter) drvMagFinal = drvMagRange;
        else if(drvMagCenter >= drvMagRange) drvMagFinal = drvMagCenter;

        //Convert drive angle from "-180 to 180" degrees to "0 to 360" degrees 
        if(drvAng < 0) { drvAng += 360;}
    } else {
        drvMagCenter = 0;
        drvMagRange = 0;
        drvMagFinal = 0;
        drvAng = 0;
        drvRot = 0;
    }
}

void GuidanceSystem::updateDashboard()
{
    frc::SmartDashboard::PutBoolean("Target Acquired", targetAcquired);
    targetAcquired = frc::SmartDashboard::PutBoolean("Target Acquired", targetAcquired);
    frc::SmartDashboard::PutNumber("Target Distance Left", distanceLeftTarget);
    distanceLeftTarget = frc::SmartDashboard::GetNumber("Target Distance Left", distanceLeftTarget);
    frc::SmartDashboard::PutNumber("Target Distance Right", distanceRightTarget);
    distanceRightTarget = frc::SmartDashboard::GetNumber("Target Distance Right", distanceRightTarget);
    frc::SmartDashboard::PutNumber("Target Width Left", widthLeftTarget);
    widthLeftTarget = frc::SmartDashboard::GetNumber("Target Width Left", widthLeftTarget);
    frc::SmartDashboard::PutNumber("Target Width Right", widthRightTarget);
    widthRightTarget = frc::SmartDashboard::GetNumber("Target Width Right", widthRightTarget);
    frc::SmartDashboard::PutNumber("Guidance Actual Height", heightRightTarget);
    heightRightTarget = frc::SmartDashboard::GetNumber("Guidance Actual Height", heightRightTarget);
    frc::SmartDashboard::PutNumber("Guidance Target Height", targetDesiredHeight);
    targetDesiredHeight = frc::SmartDashboard::GetNumber("Guidance Target Height", targetDesiredHeight);
    frc::SmartDashboard::PutNumber("Guidance kp Center Mag", kpCenterMag);
    kpCenterMag = frc::SmartDashboard::GetNumber("Guidance kp Center Mag", kpCenterMag);
    frc::SmartDashboard::PutNumber("Guidance kp Center Angle", kpCenterAng);
    kpCenterAng = frc::SmartDashboard::GetNumber("Guidance kp Center Angle", kpCenterAng);
    frc::SmartDashboard::PutNumber("Guidance kp Angle", kpAngle);
    kpAngle = frc::SmartDashboard::GetNumber("Guidance kp Angle", kpAngle);
    frc::SmartDashboard::PutNumber("Guidance kp Range", kpRange);
    kpRange = frc::SmartDashboard::GetNumber("Guidance kp Range", kpRange);

    frc::SmartDashboard::PutNumber("Guidance error Center", errorCenter);
    frc::SmartDashboard::PutNumber("Guidance error Angle", errorAngle);
    frc::SmartDashboard::PutNumber("Guidance error Range", errorHeight);
    
    frc::SmartDashboard::PutNumber("Guidance Drv Mag Center", drvMagCenter);
    frc::SmartDashboard::PutNumber("Guidance Drv Mag Range" , drvMagRange);
    frc::SmartDashboard::PutNumber("Guidnace Drv Angle", drvAng);
    frc::SmartDashboard::PutNumber("Guidnace Drv Rotation", drvRot);
}