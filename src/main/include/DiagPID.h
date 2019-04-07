#ifndef SRC_DiagPID_H_
#define SRC_DiagPID_H_ 1

#include <cmath>

class DiagPID{
    public:

    enum class PIDType{
        kPosition,
        kVelocity
    };

    DiagPID(double enableThreshold, double failSampleThreshold, double failCountThreshold, PIDType type);
    bool process(double processValue, double outputCommand);
    bool isFailed();
    void reset();

    private:   

    void initialize();
    
    PIDType _type;
    bool _isFailed;
    double _enableThreshold;
    double _failSampleThreshold;
    double _failCountThreshold;

    double _processValuePrev;
    int _failCounter;
};


#endif /* SRC_DiagPID_H_ */