#include "IO.h"
#include "frc/WPILib.h"


class LiftPIDControl {
    public:
    LiftPIDControl(IO *io);

    enum class liftPos{
    RETRACTED,
    EXTENDEDLEVELONE,
    EXTENDEDLEVELTWO
    };

  liftPos liftFrontPosDes, liftFrontPosAct, liftRearPosDes, liftRearPosAct;
  bool moveFast, syncFrontRearLifts;

  void liftPIDControlTeleopPeriodic();
  void liftPIDControlRobotPeriodic();

private:

  IO *_io;
  frc::PIDController *liftlfPID, *liftrfPID, *liftlbPID, *liftrbPID;

  double liftPIDp, liftPIDi, liftPIDd, liftPIDf, liftPIDPeriod;
  double liftFrontSetPoint, liftRearSetPoint;
  double liftPosRetractedFront, liftPosExtendedLevelOneFront, liftPosExtendedLevelTwoFront;
  double liftPosRetractedRear, liftPosExtendedLevelOneRear, liftPosExtendedLevelTwoRear;
  double liftMovementRate;
  double liftDesyncDistanceThreshold;

  double rampToValue(double currVal, double desVal, double _rampRate);
  void disableLiftPID();

};