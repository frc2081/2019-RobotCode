#include "SwerveLib.h"
#include "ControllerManager.h"
#include "IO.h"
#include "RobotCommands.h"
#include "frc/WPILib.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class GuidanceSystem{
    public:
        GuidanceSystem(RobotCommands *cmds);
        void GuidanceSystemPeriodic();
        void GuidanceSystemRobotPeriodic();

    private:
        RobotCommands *_cmds;
        void calcHomingVectors();
        void updateDashboard();

        std::shared_ptr<NetworkTable> table; 

        enum class GuidanceSysState {
            NOTACTIVE,
            TRACKING,
            FINALAPPROACH,
            PLACEMENT,
            BACKAWAY,
            DONE
        };

        GuidanceSysState GuidanceState;
        
        //Timers and durations
        //All times in ms, divided by 20 to convert into control system loops (20ms each)
        int finalApproachDuration = 500 / 20;
        int placementDuration = 500 / 20;
        int backAwayDuration = 500 / 20;
        int guidanceCompleteDelay = 3000  / 20;

        int timer;

        //Placeholder target data from vision system
        //All values in image pixels
        bool targetAcquired = false;
        int visionHeartbeat = -1;
        int distanceLeftTarget = -80;
        int distanceRightTarget = 50;
        int widthLeftTarget = 10;
        int widthRightTarget = 55;
        int heightLeftTarget = 55;
        int heightRightTarget = 110;
        
        //Height of the targets in pixels when the robot has moved to the hatch placement final approach position
        int targetDesiredHeight = 130;

        //Indicates if guidance system is in control of the robot
        bool guidanceSysActive = false;

        //Controls if homing seeks on one parameter at a time or all at once
        bool homingInStages = false;

        //Tolerance for previous homing stage to be considered complete when homing in stages
        double homingErrorTolerance = 10;

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

        //final approach and back away drive values
        double finalApproachDrvMag = .3;
        double finalApproachDrvAng = 0;
        double finalApproachDrvRot = 0;
};

