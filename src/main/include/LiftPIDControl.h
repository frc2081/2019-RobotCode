#include "IO.h"
#include "frc/WPILib.h"


class LiftPIDControl {
    public:
    LiftPIDControl(IO *RioIO);

    enum class liftPos{
    RETRACTED,
    EXTENDEDLEVELONE,
    EXTENDEDLEVELTWO
    };

  liftPos liftFrontPosDes, liftFrontPosAct, liftRearPosDes, liftRearPosAct;
  bool moveFast, syncFrontRearLifts;

  void liftPIDControlPeriodic();

private:
  frc::PIDController *liftlfPID;
  frc::PIDController *liftrfPID;
  frc::PIDController *liftlbPID;
  frc::PIDController *liftrbPID;

  double liftPIDp, liftPIDi, liftPIDd, liftPIDf, liftPIDPeriod;
  double liftFrontSetPoint, liftRearSetPoint;
  double liftPosRetractedFront, liftPosExtendedLevelOneFront, liftPosExtendedLevelTwoFront;
  double liftPosRetractedRear, liftPosExtendedLevelOneRear, liftPosExtendedLevelTwoRear;
  double liftMovementRate;
  double liftDesyncDistanceThreshold;

  double rampToValue(double currVal, double desVal, double _rampRate);
  void disableLiftPID();

};