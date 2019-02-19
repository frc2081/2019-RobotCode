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

    table = nt::NetworkTableInstance::GetDefault().GetTable("datatable");

    //Init Smartdashboard cal values
    frc::SmartDashboard::PutNumber("Guide Homing Stages Err Tol", homingErrorTolerance);
    frc::SmartDashboard::PutBoolean("Guide Homing In Stages", homingInStages);
    frc::SmartDashboard::PutNumber("Guide Back Away Time", placementDuration);
    frc::SmartDashboard::PutNumber("Guide Placement Time", placementDuration);
    frc::SmartDashboard::PutNumber("Guide Final Approach Time", finalApproachDuration);
    frc::SmartDashboard::PutNumber("Guide kp Range", kpRange);
    frc::SmartDashboard::PutNumber("Guide kp Angle", kpAngle);
    frc::SmartDashboard::PutNumber("Guide kp Center Angle", kpCenterAng);
    frc::SmartDashboard::PutNumber("Guide kp Center Mag", kpCenterMag);
    frc::SmartDashboard::PutNumber("Guide Center Offset", centerOffset);
    }

void GuidanceSystem::GuidanceSystemPeriodic(){

    //Set the guidance system active if any auto command is active
    if(_cmds->autoHatchPickup || _cmds->autoPlaceHatchOne ||
        _cmds->autoPlaceHatchTwo ||  _cmds->autoPlaceCargoInShip ||
       _cmds->autoPlaceCargoRocketOne || _cmds->autoPlaceCargoRocketTwo || _cmds->autoAlign) 
       {guidanceSysActive = true;}
    else {guidanceSysActive = false;}

    //Guidance system state machine
    switch(GuidanceState){
        case GuidanceSysState::NOTACTIVE:
            /*Determine if any command that uses the guidance system is present.
            If one is and the vision system has a target, set the matching elevator 
            command true for one loop to get the elevator system into placement position
            and then begin homing on the target*/

            _cmds->guidanceSysActive = false; //release control of the drivetrain if guidance is aborted.

            if(targetAcquired) { calcHomingVectors(); }

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
            applyDriveCommands();
            //Set all elevator commands false to hold the elevator system in
            //place until the robot is ready to place the game piece.
           // _cmds->hatchPickup = false;
           // _cmds->placeHatchOne = false;
           // _cmds->placeHatchTwo = false;
           // _cmds->placeCargoInShip = false;
           // _cmds->placeCargoRocketOne = false;
            //_cmds->placeCargoRocketTwo = false;

            //Exit when robot has approached close enough to lose target tracking
            if(guidanceSysActive == false || targetAcquired == false) {GuidanceState = GuidanceSysState::NOTACTIVE;}
            else if (targetAcquired == false || checkError()) {GuidanceState = GuidanceSysState::TRACKING;}
            break;
        
        //Timed drive forward to close the last distance with the target after the vision system has lost tracking
        case GuidanceSysState::FINALAPPROACH:
            timer++;
            drvAng = finalApproachDrvAng;
            drvMagFinal = finalApproachDrvMag;
            drvRot = finalApproachDrvRot;
            applyDriveCommands();
            
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
            applyDriveCommands();

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
            applyDriveCommands();

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
}

void GuidanceSystem::calcHomingVectors() {
    //If a target is present, calculate drive vectors to navigate robot to the target
    drvMagCenter = 0;
    drvMagRange = 0;
    drvMagFinal = 0;
    drvAng = 0;
    drvRot = 0;

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
        /*if(homingInStages && fabs(errorCenter < homingErrorTolerance)) {
            errorAngle = (widthLeftTarget - widthRightTarget);
            drvRot = errorAngle * kpAngle;
        }*/

        //Drive the robot forward until the targets match a
        //predetermined desires height that puts the robot at the
        //correct distance from the target
            errorHeight = (targetDesiredHeight - heightRightTarget);
            drvMagRange = errorHeight * kpRange;
        

        //Select final drive mag to be larger of the two requested values
        //This ensures that the drv mag will always be non-zero until both
        //errors have reached 0
        if(fabs(drvMagRange) >= fabs(drvMagCenter)) drvMagFinal = drvMagRange;
        else if(fabs(drvMagCenter) >= fabs(drvMagRange)) drvMagFinal = drvMagCenter;

        //Limit drive angle commands
        if(drvAng > 90) drvAng = 90;
        else if (drvAng< -90) drvAng = -90;
        //Convert drive angle from "-180 to 180" degrees to "0 to 360" degrees 
        if(drvAng < 0) { drvAng += 360;}
    } 
}

void GuidanceSystem::GuidanceSystemRobotPeriodic(){
    updateDashboard();
}

void GuidanceSystem::updateDashboard()
{
    targetAcquired = table->GetBoolean("TargetDataValid", false);
    visionHeartbeat = table->GetNumber("VisionHeartbeat", -2);
    distanceLeftTarget = table->GetNumber("LeftTargetDistFromCenter", -2);
    distanceRightTarget = table->GetNumber("RightTargetDistFromCenter", -2);
    widthLeftTarget = table->GetNumber("LeftTargetWidth", -2);
    widthRightTarget = table->GetNumber("RightTargetWidth", -2);
    heightLeftTarget = table->GetNumber("LeftTargetHeight", -2);
    heightRightTarget = table->GetNumber("RightTargetHeight", -2);
    
    frc::SmartDashboard::PutBoolean("Target Acquired", targetAcquired);
    frc::SmartDashboard::PutNumber("Target Vision Heartbeat", visionHeartbeat);
    frc::SmartDashboard::PutNumber("Target Width Left", widthLeftTarget);
    frc::SmartDashboard::PutNumber("Target Width Right", widthRightTarget);
    frc::SmartDashboard::PutNumber("Target Right Height", heightRightTarget);
    frc::SmartDashboard::PutNumber("Target Left Height", heightLeftTarget);
    frc::SmartDashboard::PutNumber("Target Target Height", targetDesiredHeight);
    frc::SmartDashboard::PutNumber("Target Distance Left", distanceLeftTarget);
    frc::SmartDashboard::PutNumber("Target Distance Right", distanceRightTarget);

    targetDesiredHeight = frc::SmartDashboard::GetNumber("Guide Desired Height", targetDesiredHeight);
    kpCenterMag = frc::SmartDashboard::GetNumber("Guidance kp Center Mag", kpCenterMag);
    kpCenterAng = frc::SmartDashboard::GetNumber("Guidance kp Center Angle", kpCenterAng);
    kpAngle = frc::SmartDashboard::GetNumber("Guidance kp Angle", kpAngle);
    kpRange = frc::SmartDashboard::GetNumber("Guidance kp Range", kpRange);
    finalApproachDuration = frc::SmartDashboard::GetNumber("Guide Final Approach Time", finalApproachDuration);
    placementDuration = frc::SmartDashboard::GetNumber("Guide Final Approach Time", placementDuration);
    backAwayDuration = frc::SmartDashboard::GetNumber("Guide Final Approach Time", backAwayDuration);
    finalApproachDrvMag = frc::SmartDashboard::GetNumber("Guide Final DrvMag tune", finalApproachDrvMag);
    homingInStages = frc::SmartDashboard::GetBoolean("Guide Homing In Stages", homingInStages);
    homingErrorTolerance = frc::SmartDashboard::GetNumber("Guide Homing Stages Err Tol", homingErrorTolerance);
    centerOffset = frc::SmartDashboard::GetNumber("Guide Center Offset", centerOffset);

    frc::SmartDashboard::PutNumber("Guide Final Drv Mag", drvMagFinal);
    frc::SmartDashboard::PutNumber("Guide error Center", errorCenter);
    frc::SmartDashboard::PutNumber("Guide error Angle", errorAngle);
    frc::SmartDashboard::PutNumber("Guide error Range", errorHeight);
    frc::SmartDashboard::PutNumber("Guide Drv Mag Center", drvMagCenter);
    frc::SmartDashboard::PutNumber("Guide Drv Mag Range" , drvMagRange);
    frc::SmartDashboard::PutNumber("Guide Drv Angle", drvAng);
    frc::SmartDashboard::PutNumber("Guide Drv Rotation", drvRot);
    frc::SmartDashboard::PutNumber("Guide System State", static_cast<int>(GuidanceState));
    frc::SmartDashboard::PutBoolean("Guide System Active", guidanceSysActive);
}

void GuidanceSystem::applyDriveCommands(){
    _cmds->guidanceSysActive = true;

    //if(drvMagFinal > maxDriveCommand) drvMagFinal = maxDriveCommand;
    //else if(drvMagFinal < -maxDriveCommand) drvMagFinal = -maxDriveCommand;
    _cmds->autodrvang = drvAng;

    //limit drivetrain speed to a level the auto system can handle
    if(_cmds->drvmag > maxDriveCommand) _cmds->drvmag = maxDriveCommand;
    _cmds->autodrvmag = _cmds->drvmag;
    _cmds->autodrvrot = _cmds->drvrot;
}

bool GuidanceSystem::checkError(){
    if(fabs(errorCenter) < desiredErrorCenter && fabs(errorHeight) < desiredErrorHeight) return true;
    else return false;
}