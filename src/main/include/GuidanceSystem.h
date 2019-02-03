#include "SwerveLib.h"
#include "ControllerManager.h"
#include "IO.h"
#include "RobotCommands.h"
#include "frc/WPILib.h"

class GuidanceSystem{
    public:
        GuidanceSystem(RobotCommands *cmds);
        void GuidanceSystemPeriodic();
        RobotCommands *_cmds;
        
        //Placeholder target data from vision system
        //All values in image pixels
        bool targetAcquired = true;
        int distanceLeftTarget = -80;
        int distanceRightTarget = 50;
        int widthLeftTarget = 10;
        int widthRightTarget = 55;
        int heightRightTarget = 110;
        
        //Height of the targets in pixels when the robot has moved to the hatch placement final approach position
        int targetDesiredHeight = 130;

        //Indicates if guidance system is in control of the robot
        bool guidanceSysActive = false;

        //Proportional Coefficients     
        double kpCenterAng = .5;
        double kpCenterMag = .5;
        double kpAngle = -.5;
        double kpRange = .5;

        //Actual Errors in image pixels
        double errorCenter;
        double errorAngle;
        double errorHeight;

        //Computed Drive commands
        double drvMagCenter;
        double drvMagRange;
        double drvMagFinal;
        
        //Desired swerve drive angle in -180 to 180 degrees
        //Note: This must be converted to 0 to 360 command
        //before being sent to swerve
        double drvAng;

        //Desired swerve rotation in radians/s
        double drvRot;

};

