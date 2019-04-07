#include "DiagPID.h"

DiagPID::DiagPID(double enableThreshold, double failSampleThreshold, double failCountThreshold, PIDType type){
    _type = type;
    _enableThreshold = enableThreshold;
    _failSampleThreshold = failSampleThreshold;
    _failCountThreshold = failCountThreshold;

    initialize();
}

bool DiagPID::process(double processValue, double outputCommand) {

    bool enabled;
    if(fabs(outputCommand) > _enableThreshold) enabled = true;
    else enabled = false;

    //If command is high enough and the process value does not change by at least the fail threshold every loop
    //the diagnostic will detect a fail sample
    if(enabled == true){
        if(fabs(_processValuePrev - processValue) < _failSampleThreshold) _failCounter++;
        else if (_failCounter > 0) _failCounter--;
    }

    //Trip the diagnostic if too many failed sample have been counter
    //Heal it if fail counter returs to 0
    if(_failCounter >= _failCountThreshold) _isFailed = true;
    else if (_failCounter == 0) _isFailed = false;

    _processValuePrev = processValue;

    //Disabling the diagnostic fully resets it
    if(enabled == false) initialize();

    return _isFailed;
}

bool DiagPID::isFailed(){
    return _isFailed;
}

void DiagPID::reset(){
    _isFailed = false;
}

void DiagPID::initialize(){
    _isFailed = false;
    _failCounter = 0;
    _processValuePrev = 0;
}

    //if motor output is more than 50% for 8 seconds