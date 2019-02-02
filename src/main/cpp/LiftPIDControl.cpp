#include "LiftPIDControl.h"

LiftPIDControl::LiftPIDControl(IO *io) {

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

  //Init lift PID control parameters
  //PID tunes are the same for all four lifts
  liftPIDp = .6;
  liftPIDi = 0;
  liftPIDd = 0;
  liftPIDf = 0;
  liftPIDPeriod = 0.05;
  liftDesyncDistanceThreshold = 1; //Limit of how differnt the lift leg heights are allowed to be before the lifts are shut down

  //Lift position constants
  liftPosRetractedFront = 0; //Lift front leg position in cm to fully retract the legs
  liftPosExtendedLevelOneFront = 11; //Lift front leg position in inches to raise the robot to Hab level 1
  liftPosExtendedLevelTwoFront = 22; //Lift front leg position in inches to raise the robot to Hab level 2
  liftPosRetractedRear = 0; //Lift rear leg position in cm to fully retract the legs
  liftPosExtendedLevelOneRear = 11; //Lift rear leg position in inches to raise the robot to Hab level 1
  liftPosExtendedLevelTwoRear = 22; //Lift rear leg position in inches to raise the robot to Hab level 2
  liftMovementRate = .05; //Speed in inches per loop that the lift should move at when raising or lowering the robot

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
  //liftlbPID->Enable();
  //liftlfPID->Enable();

  liftPIDp = frc::SmartDashboard::PutNumber("Climb P", liftPIDp);
  liftPIDi = frc::SmartDashboard::PutNumber("Climb I", liftPIDi);
  liftPIDd = frc::SmartDashboard::PutNumber("Climb D", liftPIDd);
  liftPIDf = frc::SmartDashboard::PutNumber("Climb F", liftPIDf);
  liftDesyncDistanceThreshold = frc::SmartDashboard::PutNumber("Climber Desync Distance", liftDesyncDistanceThreshold);
  liftPosRetractedFront = frc::SmartDashboard::PutNumber("Lift Front Retracted Set Point", liftPosRetractedFront);
  liftPosExtendedLevelOneFront = frc::SmartDashboard::PutNumber("Lift Front Hab Level 1 Set Point", liftPosExtendedLevelOneFront);
  liftPosExtendedLevelTwoFront = frc::SmartDashboard::PutNumber("Lift Front Hab Level 2 Set Point", liftPosExtendedLevelTwoFront);
  liftPosRetractedRear = frc::SmartDashboard::PutNumber("Lift Rear Retracted Set Point", liftPosRetractedRear);
  liftPosExtendedLevelOneRear = frc::SmartDashboard::PutNumber("Lift Rear Hab Level 1 Set Point", liftPosExtendedLevelOneRear);
  liftPosExtendedLevelTwoRear = frc::SmartDashboard::PutNumber("Lift Rear Hab Level 2 Set Point", liftPosExtendedLevelTwoRear);
  liftMovementRate = frc::SmartDashboard::PutNumber("Lift Movement Rate", liftMovementRate);
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
/*
  liftBLPID->SetP(liftPIDp);
  liftBLPID->SetI(liftPIDi);
  liftBLPID->SetD(liftPIDd);
  liftBLPID->SetF(liftPIDf);

  liftBRPID->SetP(liftPIDp);
  liftBRPID->SetI(liftPIDi);
  liftBRPID->SetD(liftPIDd);
  liftBRPID->SetF(liftPIDf);*/

   //liftDrive->Set(-stick->GetY());
  
  //Basic tuning code
 // liftFrontSetPoint += stick->GetY(frc::GenericHID::JoystickHand::kLeftHand) * 50;
  //if(liftFrontSetPoint < 0) liftFrontSetPoint = 0;

  //Position Testing Code
  
  /*if(stick->GetAButton()) { liftFrontPosDes = liftPos::RETRACTED; moveFast = false; }
  else if(stick->GetBButton()) { liftFrontPosDes = liftPos::EXTENDEDLEVELONE; moveFast = false; }
  else if(stick->GetXButton()) { liftFrontPosDes = liftPos::EXTENDEDLEVELTWO; moveFast = false;}
  else if(stick->GetYButton()) { liftFrontPosDes = liftPos::RETRACTED; moveFast = true; }
  else if(stick->GetStartButton()) { liftFrontPosDes = liftPos::EXTENDEDLEVELONE; moveFast = true;}
  else if(stick->GetBackButton()) { liftFrontPosDes = liftPos::EXTENDEDLEVELTWO; moveFast = true;}*/
  
  /*Convert Commands into position setpoint commands for the lift PID controls
  Need to allow fast movement of the racks when they are simply traveling to new positions and slow careful movement when they are
  actually lifting the robot
  So if fast movement is desired, set the setpoint directly to the destination position and let the PID move as fast as it can
  else if fast movement is not wanted, slowly ramp the PID setpoint to the destination position to slowly move the racks
  */
  if(liftFrontPosDes == liftPos::RETRACTED && moveFast == true) liftFrontSetPoint = liftPosRetractedFront;
  else if(liftFrontPosDes == liftPos::EXTENDEDLEVELONE && moveFast == true) liftFrontSetPoint = liftPosExtendedLevelOneFront;
  else if(liftFrontPosDes == liftPos::EXTENDEDLEVELTWO && moveFast == true) liftFrontSetPoint = liftPosExtendedLevelTwoFront;
  else if(liftFrontPosDes == liftPos::RETRACTED) liftFrontSetPoint = rampToValue(liftFrontSetPoint, liftPosRetractedFront, liftMovementRate);
  else if(liftFrontPosDes == liftPos::EXTENDEDLEVELONE) liftFrontSetPoint = rampToValue(liftFrontSetPoint, liftPosExtendedLevelOneFront, liftMovementRate);
  else if(liftFrontPosDes == liftPos::EXTENDEDLEVELTWO) liftFrontSetPoint = rampToValue(liftFrontSetPoint, liftPosExtendedLevelTwoFront, liftMovementRate);
  else liftFrontSetPoint = liftPosRetractedFront; //Default Case

  liftrfPID->SetSetpoint(liftFrontSetPoint);
  liftlfPID->SetSetpoint(liftFrontSetPoint);
  /*liftlfPID->SetSetpoint(liftFrontSetPoint);
  liftlbPID->SetSetpoint(liftRearSetPoint);
  liftrbPID->SetSetpoint(liftRearSetPoint);*/

  //Safety code to stop the lifts if they get out of sync with each other
  //Might improve this in the future to keep them running and limit command so they stay in sync
  /*double rfPos = liftrfenc->GetDistance();
  double lfPos = liftlfenc->GetDistance();
  double rbPos = liftrbenc->GetDistance();
  double lbPos = liftlbenc->GetDistance();
  if(abs(rfPos - lfPos) > liftDesyncDistanceThreshold) {disableLiftPID();}
  if(abs(rbPos - lbPos) > liftDesyncDistanceThreshold) {disableLiftPID();}*/
}

double LiftPIDControl::rampToValue(double currVal, double desVal, double rampRate){
  double _rampRate = fabs(rampRate); //Following code does not work if rampRate is set to a negative value
  if(desVal > currVal && currVal + _rampRate > desVal) return desVal;
  else if(desVal < currVal && currVal - _rampRate < desVal) return desVal;
  else if(desVal > currVal) return currVal + _rampRate;
  else if(desVal < currVal) return currVal - _rampRate;
  else return currVal; //Default case
}

void LiftPIDControl::disableLiftPID() {
  liftlfPID->Disable();
  liftlfPID->Disable();
  liftlfPID->Disable();
  liftlfPID->Disable();
}