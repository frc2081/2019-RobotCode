#include "LiftPIDControl.h"

LiftPIDControl::LiftPIDControl(IO *io, RobotCommands *cmds) {

  _cmds = cmds;
  _io = io;
   //Init lift movement commands and actual positions
  liftFrontPosDes = liftPos::RETRACTED;
  liftFrontPosAct = liftPos::RETRACTED;
  liftRearPosDes = liftPos::RETRACTED;
  liftRearPosAct = liftPos::RETRACTED;
  moveFast = false; //Commands the lift legs to move rapidly or slowly. Rapid movement used when lift legs are not supporting the robot.
  syncFrontRearLifts = false; //Commands the front and rear lifts to move in sync.
  liftFrontSetPoint = 0;//Increasing value = extending the lift legs
  liftRearSetPoint = 0;
  liftDestinationRear = 0;
  liftDestinationFront = 0;

  freezeSetPointFront = 0;
  freezeSetPointRear = 0;
  frozen = false;

  //Init lift PID control parameters
  //PID tunes are the same for all four lifts
  liftPIDp = 1;
  liftPIDi = 0;
  liftPIDd = 0;
  liftPIDf = 0;
  liftPIDPeriod = 0.05;
  liftDesyncDistanceThreshold = 1; //Limit of how differnt the lift leg heights are allowed to be before the lifts are shut down
  liftPosTolerance = .5; //How close lift will attempt to get to the desired position

  //Lift position constants
  liftPosRetractedFront = 0; //Lift front leg position in cm to fully retract the legs
  liftPosExtendedLevelOneFront = 9; //Lift front leg position in inches to raise the robot to Hab level 1
  liftPosExtendedLevelTwoFront = 22; //Lift front leg position in inches to raise the robot to Hab level 2
  liftPosRetractedRear = 0; //Lift rear leg position in cm to fully retract the legs
  liftPosExtendedLevelOneRear = 9; //Lift rear leg position in inches to raise the robot to Hab level 1
  liftPosExtendedLevelTwoRear = 22; //Lift rear leg position in inches to raise the robot to Hab level 2
  liftMovementRate = .15; //Speed in inches per loop that the lift should move at when raising or lowering the robot

  liftrfPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, _io->liftrfenc, _io->liftrfmot, liftPIDPeriod);
  liftlfPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, _io->liftlfenc, _io->liftlfmot, liftPIDPeriod);
  liftlbPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, _io->liftlbenc, _io->liftlbmot, liftPIDPeriod);
  liftrbPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, _io->liftrbenc, _io->liftrbmot, liftPIDPeriod);
  
  liftrfPID->SetPercentTolerance(3);
  liftlfPID->SetPercentTolerance(3);
  liftrbPID->SetPercentTolerance(3);
  liftlbPID->SetPercentTolerance(3);

  liftrfPID->Enable(); 
  liftlfPID->Enable();
  liftlbPID->Enable();
  liftrbPID->Enable();

  frc::SmartDashboard::PutNumber("Climb P", liftPIDp);
  frc::SmartDashboard::PutNumber("Climb I", liftPIDi);
  frc::SmartDashboard::PutNumber("Climb D", liftPIDd);
  frc::SmartDashboard::PutNumber("Climb F", liftPIDf);
  frc::SmartDashboard::PutNumber("Climber Desync Distance", liftDesyncDistanceThreshold);
  frc::SmartDashboard::PutNumber("Lift Front Retracted Set Point", liftPosRetractedFront);
  frc::SmartDashboard::PutNumber("Lift Front Hab Level 1 Set Point", liftPosExtendedLevelOneFront);
  frc::SmartDashboard::PutNumber("Lift Front Hab Level 2 Set Point", liftPosExtendedLevelTwoFront);
  frc::SmartDashboard::PutNumber("Lift Rear Retracted Set Point", liftPosRetractedRear);
  frc::SmartDashboard::PutNumber("Lift Rear Hab Level 1 Set Point", liftPosExtendedLevelOneRear);
  frc::SmartDashboard::PutNumber("Lift Rear Hab Level 2 Set Point", liftPosExtendedLevelTwoRear);
  frc::SmartDashboard::PutNumber("Lift Movement Rate", liftMovementRate);
  frc::SmartDashboard::PutBoolean("Lift Freeze", frozen);

}    

void LiftPIDControl::liftPIDControlRobotPeriodic(){

//Get all dashboard-adjustable cal values and set them
  liftPIDp = frc::SmartDashboard::GetNumber("Climb P", liftPIDp);
  liftPIDi = frc::SmartDashboard::GetNumber("Climb I", liftPIDi);
  liftPIDd = frc::SmartDashboard::GetNumber("Climb D", liftPIDd);
  liftPIDf = frc::SmartDashboard::GetNumber("Climb F", liftPIDf);
  liftDesyncDistanceThreshold = frc::SmartDashboard::GetNumber("Climber Desync Distance", liftDesyncDistanceThreshold);
  liftPosRetractedFront = frc::SmartDashboard::GetNumber("Lift Front Retracted Set Point", liftPosRetractedFront);
  liftPosExtendedLevelOneFront = frc::SmartDashboard::GetNumber("Lift Front Hab Level 1 Set Point", liftPosExtendedLevelOneFront);
  liftPosExtendedLevelTwoFront = frc::SmartDashboard::GetNumber("Lift Front Hab Level 2 Set Point", liftPosExtendedLevelTwoFront);
  liftPosRetractedRear = frc::SmartDashboard::GetNumber("Lift Rear Retracted Set Point", liftPosRetractedRear);
  liftPosExtendedLevelOneRear = frc::SmartDashboard::GetNumber("Lift Rear Hab Level 1 Set Point", liftPosExtendedLevelOneRear);
  liftPosExtendedLevelTwoRear = frc::SmartDashboard::GetNumber("Lift Rear Hab Level 2 Set Point", liftPosExtendedLevelTwoRear);
  liftMovementRate = frc::SmartDashboard::GetNumber("Lift Movement Rate", liftMovementRate);

  //Write all data to dashboard
  frc::SmartDashboard::PutNumber("Climb P", liftPIDp);
  frc::SmartDashboard::PutNumber("Climb I", liftPIDi);
  frc::SmartDashboard::PutNumber("Climb D", liftPIDd);
  frc::SmartDashboard::PutNumber("Climb F", liftPIDf);
  frc::SmartDashboard::PutNumber("Climb Front Set Point", liftFrontSetPoint);
  frc::SmartDashboard::PutNumber("Climb Rear Set Point", liftRearSetPoint);
  frc::SmartDashboard::PutNumber("Climber RF PID Actual SetPoint", liftrfPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber LF PID Actual SetPoint", liftlfPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber LB PID Actual SetPoint", liftlbPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber RB PID Actual SetPoint", liftrbPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber RF Actual Position", _io->liftrfenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber LF Actual Position", _io->liftlfenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber LB Actual Position", _io->liftlbenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber RB Actual Position", _io->liftrbenc->GetDistance());
  frc::SmartDashboard::PutBoolean("Fast Movement", moveFast);
  frc::SmartDashboard::PutNumber("Climber Desync Distance", liftDesyncDistanceThreshold);
  frc::SmartDashboard::PutNumber("Lift Front Retracted Set Point", liftPosRetractedFront);
  frc::SmartDashboard::PutNumber("Lift Front Hab Level 1 Set Point", liftPosExtendedLevelOneFront);
  frc::SmartDashboard::PutNumber("Lift Front Hab Level 2 Set Point", liftPosExtendedLevelTwoFront);
  frc::SmartDashboard::PutNumber("Lift Rear Retracted Set Point", liftPosRetractedRear);
  frc::SmartDashboard::PutNumber("Lift Rear Hab Level 1 Set Point", liftPosExtendedLevelOneRear);
  frc::SmartDashboard::PutNumber("Lift Rear Hab Level 2 Set Point", liftPosExtendedLevelTwoRear);
  frc::SmartDashboard::PutNumber("Lift Movement Rate", liftMovementRate);
  frc::SmartDashboard::PutNumber("Lift LF Motor Power", _io->liftlfmot->Get());
  frc::SmartDashboard::PutNumber("Lift RF Motor Power", _io->liftrfmot->Get());
  frc::SmartDashboard::PutNumber("Lift LB Motor Power", _io->liftlbmot->Get());
  frc::SmartDashboard::PutNumber("Lift RB Motor Power", _io->liftrbmot->Get());
  frc::SmartDashboard::PutNumber("Lift Frozen", frozen);
}

void LiftPIDControl::liftPIDControlTeleopPeriodic() {

  liftrfPID->SetP(liftPIDp);
  liftrfPID->SetI(liftPIDi);
  liftrfPID->SetD(liftPIDd);
  liftrfPID->SetF(liftPIDf);

  liftlfPID->SetP(liftPIDp);
  liftlfPID->SetI(liftPIDi);
  liftlfPID->SetD(liftPIDd);
  liftlfPID->SetF(liftPIDf);

  liftlbPID->SetP(liftPIDp);
  liftlbPID->SetI(liftPIDi);
  liftlbPID->SetD(liftPIDd);
  liftlbPID->SetF(liftPIDf);

  liftrbPID->SetP(liftPIDp);
  liftrbPID->SetI(liftPIDi);
  liftrbPID->SetD(liftPIDd);
  liftrbPID->SetF(liftPIDf);
  
  /*Convert Commands into position setpoint commands for the lift PID controls
  Need to allow fast movement of the racks when they are simply traveling to new positions and slow careful movement when they are
  actually lifting the robot
  So if fast movement is desired, set the setpoint directly to the destination position and let the PID move as fast as it can
  else if fast movement is not wanted, slowly ramp the PID setpoint to the destination position to slowly move the racks
*/

  liftDestinationFront = setLiftDestination(liftFrontPosDes);
  liftDestinationRear = setLiftDestination(liftRearPosDes);

  if(moveFast == false){
    liftFrontSetPoint = rampToValue(liftFrontSetPoint, liftDestinationFront, liftMovementRate);
    liftRearSetPoint = rampToValue(liftRearSetPoint, liftDestinationRear, liftMovementRate);
  } else {
    liftFrontSetPoint = liftDestinationFront;
    liftRearSetPoint = liftDestinationRear;
  }

  double rfPos = _io->liftrfenc->GetDistance();
  double lfPos = _io->liftlfenc->GetDistance();
  double rbPos = _io->liftrbenc->GetDistance();
  double lbPos = _io->liftlbenc->GetDistance();
  
  //Determine if the lifts have reached their current destination
  if(liftOnTarget(liftDestinationFront, rfPos, liftPosTolerance) && liftOnTarget(liftDestinationFront, lfPos, liftPosTolerance)) liftFrontPosAct = liftFrontPosDes;
  if(liftOnTarget(liftDestinationRear, rbPos, liftPosTolerance) && liftOnTarget(liftDestinationRear, lbPos, liftPosTolerance)) liftRearPosAct = liftRearPosDes;

  //Safety code to stop the lifts if they get out of sync with each other
  //Might improve this in the future to keep them running and limit command so they stay in sync
  double liftFrontSeparation = fabs(rfPos - lfPos);
  double liftRearSeparation = fabs(rbPos - lbPos);
  frc::SmartDashboard::PutNumber("Lift Front Separation", liftFrontSeparation);
  frc::SmartDashboard::PutNumber("Lift Rear Separation", liftRearSeparation);
  if(liftFrontSeparation > liftDesyncDistanceThreshold || liftRearSeparation > liftDesyncDistanceThreshold) {_cmds->climbFreeze = true;}

  //implement climb freeze command - freezes lift setpoints at the moment the command is issued
  if(_cmds->climbFreeze == true && frozen == false){
    freezeSetPointFront = liftFrontSetPoint;
    freezeSetPointRear = liftRearSetPoint;
    frozen = true;
  }

  if(frozen){
    liftFrontSetPoint = freezeSetPointFront;
    liftRearSetPoint = freezeSetPointRear;
  }

  liftrfPID->SetSetpoint(liftFrontSetPoint);
  liftlfPID->SetSetpoint(liftFrontSetPoint);
  liftlbPID->SetSetpoint(liftRearSetPoint);
  liftrbPID->SetSetpoint(liftRearSetPoint);
}


double LiftPIDControl::rampToValue(double currVal, double desVal, double rampRate){
  double _rampRate = fabs(rampRate); //Following code does not work if rampRate is set to a negative value
  if(desVal > currVal && currVal + _rampRate > desVal) return desVal;
  else if(desVal < currVal && currVal - _rampRate < desVal) return desVal;
  else if(desVal > currVal) return currVal + _rampRate;
  else if(desVal < currVal) return currVal - _rampRate;
  else return currVal; //Default case
}

double LiftPIDControl::setLiftDestination(liftPos liftDest) {
  double liftDesPos = 0;
  if(liftDest == liftPos::RETRACTED) liftDesPos = liftPosRetractedFront;
  else if(liftDest == liftPos::EXTENDEDLEVELONE) liftDesPos = liftPosExtendedLevelOneFront;
  else if(liftDest == liftPos::EXTENDEDLEVELTWO) liftDesPos = liftPosExtendedLevelTwoFront;
  else liftDesPos = liftPosRetractedFront;

  return liftDesPos;
}

bool LiftPIDControl::liftOnTarget(double targetPos, double actualPos, double tolerance) {
  if(actualPos < targetPos + tolerance && actualPos > targetPos - tolerance) return true;
  else return false;
}