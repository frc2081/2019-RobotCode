#include "GuidanceSystem.h"

GuidanceSystem::GuidanceSystem(RobotCommands *cmds){
    _cmds = cmds;
}

void GuidanceSystem::GuidanceSystemPeriodic(){
    errorDist = (distanceLeft + distanceRight) * kpDist;
    if(errorDist < 0){
        _cmds->drvang = moveRight;
    
    }else if (errorDist > 0) {
        _cmds->drvang = moveLeft;
    }
    errorAngle = (widthLeft - widthRight) *kpAngle;
    if(errorAngle > 0){
        _cmds->drvrot = turnLeft;
    
    }else if (errorAngle < 0){
        _cmds->drvrot = turnRight;
    }
    errorMag = (targetHeight - actualHeight)/targetHeight;
    _cmd->driveMag = errorMag;


    frc::SmartDashboard::PutNumber("distance Left", distanceLeft);
    distanceLeft = frc::SmartDashboard::GetNumber("distance Left", -80);
    
    frc::SmartDashboard::PutNumber("distance Right", distanceRight);
    distanceRight = frc::SmartDashboard::GetNumber("distance Right", 50);
    
    frc::SmartDashboard::PutNumber("width Left", widthLeft);
    widthLeft = frc::SmartDashboard::GetNumber("widthLeft", 10);
    
    frc::SmartDashboard::PutNumber("width Right", widthRight);
    widthRight = frc::SmartDashboard::GetNumber("widthRight", 55);

    frc::SmartDashboard::PutNumber("move Right", moveRight);
    moveRight = frc::SmartDashboard::GetNumber("moveRight", 90);

    frc::SmartDashboard::PutNumber("move Left", moveLeft);
    moveLeft = frc::SmartDashboard::GetNumber("moveLeft", 270);

    frc::SmartDashboard::PutNumber("kp Dist", kpDist);
    kpDist = frc::SmartDashboard::GetNumber("kpDist", 120);

    frc::SmartDashboard::PutNumber("kp Angle", kpAngle);
    kpAngle= frc::SmartDashboard::GetNumber("kpAngle", 110);

    frc::SmartDashboard::PutNumber("target Height", targetHeight);
    targetHeight= frc::SmartDashboard::GetNumber("targetHeight", 130);

    frc::SmartDashboard::PutNumber("actual Height", actualHeight);
    actualHeight= frc::SmartDashboard::GetNumber("actualHeight", 110);

    frc::SmartDashboard::PutNumber("turn Left", turnLeft);
    turnLeft= frc::SmartDashboard::GetNumber("turnLeft", -0.5);

    frc::SmartDashboard::PutNumber("turn Right", turnRight);
    turnRight= frc::SmartDashboard::GetNumber("turnRight", 0.5);

    frc::SmartDashboard::PutNumber("error Dist", errorDist);

    frc::SmartDashboard::PutNumber("error Angle", errorAngle);

    frc::SmartDashboard::PutNumber("error Mag", errorMag);


    )

    
}