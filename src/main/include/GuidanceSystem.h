#include "SwerveLib.h"
#include "ControllerManager.h"
#include "IO.h"
#include "RobotCommands.h"
#include "frc/WPILib.h"

class GuidanceSystem{
    public:
        GuidanceSystem(RobotCommands *cmds);
        RobotCommands *_cmds;
        int distanceLeft = -80;
        int distanceRight = 50;
        int widthLeft = 10;
        int widthRight = 55;
        int moveRight = 90;
        int moveLeft = 270;
        int kpDist = 120;
        int kpAngle = 110;
        int targetHeight = 130;
        int actualHeight =110;
        double turnLeft = -0.5;
        double turnRight = 0.5;
        double errorDist;
        double errorAngle;
        double errorMag;
}

