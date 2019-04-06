#ifndef SRC_DiagMasterAlarm_H_
#define SRC_DiagMasterAlarm_H_ 1

#include "frc/WPILib.h"
#include "RobotCommands.h"
#include "IO.h"

class DiagMasterAlarm{
    public:
    DiagMasterAlarm(RobotCommands *Commands, IO *io);

    bool DiagMasterAlarmPeriodic();

    private:
    RobotCommands *_cmds;
    IO *_io;

    frc::Watchdog *watchDog;
    const int loopTime = 20; //control system loop time in mS
    const int blinkPeriod = 5; //Period in loops for master alarm lamp to blink. 1000ms / 20 ms loop rate * 5 loop period = 5hZ blinking 
    int blinkLoopCounter;
    bool lampState;
};

#endif /* SRC_DiagMasterAlarm_H_ */