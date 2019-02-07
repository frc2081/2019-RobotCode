/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"
#include "ctre/Phoenix.h"

IO::IO() {
	//Swerve Drive Motors
	drvrfmot = new frc::VictorSP(0);		
	drvlfmot = new frc::VictorSP(1);
	drvrbmot = new frc::VictorSP(2);
	drvlbmot = new frc::VictorSP(3);
	turnrfmot = new frc::VictorSP(4);
	turnlfmot = new frc::VictorSP(5);
	turnrbmot = new frc::VictorSP(6);
	turnlbmot = new frc::VictorSP(7);
	//Other Motors
	liftdrivemot = new frc::VictorSP(8);	
	elevatormot = new frc::VictorSP(9);

	liftrfmot = new frc::VictorSP(10);	
	liftlfmot = new frc::VictorSP(11);
	liftrbmot = new frc::VictorSP(12);
	liftlbmot = new frc::VictorSP(13);
	//PWM 14-18 used as DIO inputs
	ballintakemot = new frc::VictorSP(19);

	//Swerve Drive Encoders
	encdrvrf= new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
	encdrvlf = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);
	encdrvrb = new frc::Encoder(4, 5, false, frc::Encoder::EncodingType::k4X);
	encdrvlb = new frc::Encoder(6, 7, false, frc::Encoder::EncodingType::k4X);	
	swerveresetone = new frc::DigitalInput(8);
	swerveresettwo = new frc::DigitalInput(9);
	//DIO 10-13 are used as PWM outputs for climb motors		
	steerencdrvrf = new frc::AnalogPotentiometer(2,360,0);	
	steerencdrvlf = new frc::AnalogPotentiometer(0,360,0);	
	steerencdrvrb = new frc::AnalogPotentiometer(3,360,0);	
	steerencdrvlb = new frc::AnalogPotentiometer(1,360,0);
	//Other Encoders and DIO
	liftrfenc = new frc::Encoder(14 ,15, false, frc::Encoder::EncodingType::k4X);
	liftlfenc = new frc::Encoder(16, 17, false, frc::Encoder::EncodingType::k4X);
	liftrbenc = new frc::Encoder(18, 19, false, frc::Encoder::EncodingType::k4X);
	liftlbenc = new frc::Encoder(20, 21, false, frc::Encoder::EncodingType::k4X);	
	liftdriveenc = new frc::Counter(22);
	elevatorenc = new frc::Encoder(24, 25, false, frc::Encoder::EncodingType::k4X);
		
	//Solenoids
	compressor = new frc::Compressor();
	ballarmsolenoidin = new frc::Solenoid(0);
	hatcharmsolenoidin = new frc::Solenoid(1);
	ballshootersolenoidin = new frc::Solenoid(2);
	hatchclawsolenoidin = new frc::Solenoid(3);
	ballarmsolenoidout = new frc::Solenoid(4);
	hatcharmsolenoidout = new frc::Solenoid(5);
	ballshootersolenoidout = new frc::Solenoid(6);
	hatchclawsolenoidout = new frc::Solenoid(7);
  
	//Configure IO
  	liftrfenc->SetDistancePerPulse(liftDistPerCountInches);
  	liftlfenc->SetDistancePerPulse(liftDistPerCountInches);
	liftrbenc->SetDistancePerPulse(liftDistPerCountInches);
  	liftlbenc->SetDistancePerPulse(liftDistPerCountInches);

	//encdrvlb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvrb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvlf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvrf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);

	elevatorenc->SetDistancePerPulse(elevatorEncoderCountsToDistanceInches);
	elevatorDesiredPos = 0;
	elevatorActualPos = 0;
}

void IO::ioPeriodic(){
	elevatorActualPos = elevatorenc->GetDistance();

	if(elevatorActualPos < elevatorDesiredPos - elevatorPosTolerance){
		elevatormot->Set(1.0);
	} else if (elevatorActualPos > elevatorDesiredPos + elevatorPosTolerance){
		elevatormot->Set(-1.0);
	} else elevatormot->Set(0.0);

	frc::SmartDashboard::PutNumber("Elevator Des Pos", elevatorDesiredPos);
	frc::SmartDashboard::PutNumber("Elevator Act Pos", elevatorActualPos);
	frc::SmartDashboard::PutNumber("Elevator Motor Power", elevatormot->Get());
}

/*void IO::robotMechanismPeriodic(ElevatorManager test_12){
	if(test_12.HatchArmPos){
		hatcharmsolenoidin->Set(true);
		hatcharmsolenoidout->Set(false);
	}
	else{
		hatcharmsolenoidin->Set(false);
		hatcharmsolenoidout->Set(true);
	}
	
}
*/


/*
void IO::robotMechanismPeriodic(){
	if(HatchArmPos == HatchArmExtended){
		hatcharmsolenoidin.set(true);
		hatcharmsolenoidout.set(false);
	}
	else{
		hatcharmsolenoidin.set(false);
		hatcharmsolenoidout.set(true);
	}
	
}
*/
