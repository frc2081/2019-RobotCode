#include "DiagMasterAlarm.h"

DiagMasterAlarm::DiagMasterAlarm(RobotCommands *Commands, IO *io){
    _cmds = Commands;
    _io = io;    
    
    lampState = false;
    blinkLoopCounter = 0;
}

bool DiagMasterAlarm::DiagMasterAlarmPeriodic(){
    if(_cmds->masterAlarmState) {
        printf("\n\n**************************************");
        printf("**************************************");
        printf("*********MASTER ALARM TRIPPED*********");
        printf("**************************************");
        printf("**************************************\n\n");   

        //Blink Lamp
        blinkLoopCounter++;
        if(blinkLoopCounter > blinkPeriod){
            lampState = !lampState;
            blinkLoopCounter = 0;
        }
    }
    _io->masterAlarmLamp->Set(lampState);
}