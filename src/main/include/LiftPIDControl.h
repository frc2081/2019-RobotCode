#pragma once

#include "IO.h"
#include "frc/WPILib.h"
#include "RobotCommands.h"


class LiftPIDControl {
    public:
    LiftPIDControl(IO *io, RobotCommands *cmds);

    enum class liftPos{
    RETRACTED,
    EXTENDEDLEVELONE,
    EXTENDEDLEVELTWO
    };

  liftPos liftFrontPosDes, liftFrontPosAct, liftRearPosDes, liftRearPosAct;
  bool moveFast, syncFrontRearLifts;
  double getFrontSetPoint();
  double getRearSetPoint();

  void liftPIDControlTeleopPeriodic();
  void liftPIDControlRobotPeriodic();

private:

  IO *_io;
  RobotCommands *_cmds;
  frc::PIDController *liftlfPID, *liftrfPID, *liftlbPID, *liftrbPID;

  double liftPIDp, liftPIDi, liftPIDd, liftPIDf, liftPIDPeriod;
  double liftDestinationFront, liftDestinationRear;
  double liftFrontSetPoint, liftRearSetPoint;
  double liftPosRetractedFront, liftPosExtendedLevelOneFront, liftPosExtendedLevelTwoFront;
  double liftPosRetractedRear, liftPosExtendedLevelOneRear, liftPosExtendedLevelTwoRear;
  double liftMovementRate;
  double liftDesyncDistanceThreshold;
  double liftPosTolerance;
  bool stopLiftManual;
  double liftFrontSeparation, liftRearSeparation;


  //variables to store climb set point at moment of climb freeze command
  double freezeSetPointFront;
  double freezeSetPointRear;
  bool frozen;

  double rampToValue(double currVal, double desVal, double _rampRate);
  void disableLiftPID();
  double setLiftDestination(liftPos lift);
  bool liftOnTarget(double targetPos, double actualPos, double tolerance);
};