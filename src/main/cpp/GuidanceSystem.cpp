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
}

void GuidanceSystem::GuidanceSystemPeriodic(){

    //Determine if any command that uses the guidance system is present
    if(_cmds->autoHatchPickup == true) {_cmds->hatchPickup = true; guidanceSysActive = true; }
    else if (_cmds->autoPlaceHatchOne == true) {_cmds->placeHatchOne = true; guidanceSysActive = true; }
    else if(_cmds->autoPlaceHatchTwo == true) {_cmds->placeHatchTwo = true; guidanceSysActive = true; }
    else if(_cmds->autoPlaceCargoInShip == true) {_cmds->placeCargoInShip = true; guidanceSysActive = true; }
    else if(_cmds->autoPlaceCargoRocketOne == true) {_cmds->placeCargoRocketOne = true; guidanceSysActive = true; }
    else if(_cmds->autoPlaceCargoRocketTwo == true) {_cmds->placeCargoRocketTwo = true; guidanceSysActive = true; }
    else { guidanceSysActive = false; }

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

    //If guidance is active, tell the drivetrain and set the auto drive commands
    if(guidanceSysActive == true){
        _cmds->guidanceSysActive = true;
        _cmds->autodrvang = drvAng;
        _cmds->autodrvmag = drvMagFinal;
        _cmds->autodrvrot = drvRot;
    }

    //Add capability to execute each stage independently
    //Add capability to travel extra distance to target after vision system loses acquisiton

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