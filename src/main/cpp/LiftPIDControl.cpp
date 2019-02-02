#include "LiftPIDControl.h"

LiftPIDControl::LiftPIDControl(IO *RioIO) {
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


  liftrfPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, RioIO->liftrfenc, RioIO->liftrfmot, liftPIDPeriod);
  liftlfPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, RioIO->liftlfenc, RioIO->liftlfmot, liftPIDPeriod);
  //liftBLPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, RioIO->liftlbenc, RioIO->liftlbmot, liftPIDPeriod);
  //liftBRPID = new frc::PIDController(liftPIDp, liftPIDi, liftPIDd, liftPIDf, RioIO->liftrbenc, RioIO->liftrbmot, liftPIDPeriod);
  
  liftrfPID->SetTolerance(3);
  liftlfPID->SetTolerance(3);
  liftrfPID->Enable(); 
  liftlfPID->Enable();
  //liftBLPID->Enable();
  //liftBRPID->Enable();

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

void LiftPIDControl::liftPIDControlPeriodic()
{
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

  //Write all data to dashboard
  frc::SmartDashboard::PutNumber("Climb P", liftPIDp);
  frc::SmartDashboard::PutNumber("Climb I", liftPIDi);
  frc::SmartDashboard::PutNumber("Climb D", liftPIDd);
  frc::SmartDashboard::PutNumber("Climb F", liftPIDf);
  frc::SmartDashboard::PutNumber("Climb Front Set Point", liftFrontSetPoint);
  frc::SmartDashboard::PutNumber("Climb Rear Set Point", liftRearSetPoint);
  frc::SmartDashboard::PutNumber("Climber RF PID Actual SetPoint", liftrfPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber LF PID Actual SetPoint", liftlfPID->GetSetpoint());
  //frc::SmartDashboard::PutNumber("Climber LB PID Actual SetPoint", liftlbPID->GetSetpoint());
  //frc::SmartDashboard::PutNumber("Climber RB PID Actual SetPoint", liftrbPID->GetSetpoint());
  frc::SmartDashboard::PutNumber("Climber RF Actual Position", RioIO->liftrfenc->GetDistance());
  frc::SmartDashboard::PutNumber("Climber LF Actual Position", RioIO->liftlfenc->GetDistance());
  //frc::SmartDashboard::PutNumber("Climber LB Actual Position", liftlbenc->GetDistance());
  //frc::SmartDashboard::PutNumber("Climber RB Actual Position", liftrbenc->GetDistance());
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