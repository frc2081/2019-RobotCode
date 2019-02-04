/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"
#include "ElevatorManager.h"

IO::IO() {

	//******************PWM assingments*****************
	//Swerve Drive Motors
	drvlbmot = new frc::VictorSP(3);
	drvrbmot  = new frc::VictorSP(2);
	drvlfmot  = new frc::VictorSP(1);
	drvrfmot  = new frc::VictorSP(0);
	turnlbmot = new frc::VictorSP(7);
	turnrbmot = new frc::VictorSP(6);
	turnlfmot = new frc::VictorSP(5);
	turnrfmot = new frc::VictorSP(4);

	//Other Motors
	ballintakemot = new frc::VictorSP(15);
	liftdrivemot = new frc::VictorSP(16);
	liftlbmot = new frc::VictorSP(17);
	liftrbmot = new frc::VictorSP(18);
	liftlfmot = new frc::VictorSP(19);
	liftrfmot = new frc::VictorSP(20);
	elevatormot = new frc::VictorSP(21); //fix port #
	
	//******************DIO assingments*****************
	//Swerve Angle Reset Inputs
	swerveresetone = new frc::DigitalInput(0);
	swerveresettwo = new frc::DigitalInput(1);

	//Swerve Drive Encoders
	encdrvlb = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);
	encdrvlf = new frc::Encoder(8, 9, false, frc::Encoder::EncodingType::k4X);
	encdrvrb = new frc::Encoder(4, 5, false, frc::Encoder::EncodingType::k4X);
	encdrvrf= new frc::Encoder(6, 7, false, frc::Encoder::EncodingType::k4X);
	//Other Encoders

	liftdriveenc = new frc::Encoder(10, 11, false, frc::Encoder::EncodingType::k4X);
	liftlbenc = new frc::Encoder(22, 21, false, frc::Encoder::EncodingType::k4X);
	liftrbenc = new frc::Encoder(13, 14, false, frc::Encoder::EncodingType::k4X);
	liftlfenc = new frc::Encoder(15, 16, false, frc::Encoder::EncodingType::k4X);
	liftrfenc = new frc::Encoder(17 ,18, false, frc::Encoder::EncodingType::k4X);
	elevatorenc = new frc::Encoder(19,20, false, frc::Encoder::EncodingType::k4X);

	//******************Analog Input assingments*****************
	steerencdrvlb = new frc::AnalogPotentiometer(1,360,0);
	steerencdrvlf = new frc::AnalogPotentiometer(0,360,0);
	steerencdrvrb = new frc::AnalogPotentiometer(3,360,0);
	steerencdrvrf = new frc::AnalogPotentiometer(2,360,0);

	encdrvlb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvrb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvlf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvrf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);

	compressor = new frc::Compressor();
	
	//Solenoids
	ballarmsolenoidin = new frc::Solenoid(0);
	hatcharmsolenoidin = new frc::Solenoid(1);
	ballshootersolenoidin = new frc::Solenoid(2);
	hatchclawsolenoidin = new frc::Solenoid(3);
	ballarmsolenoidout = new frc::Solenoid(4);
	hatcharmsolenoidout = new frc::Solenoid(5);
	ballshootersolenoidout = new frc::Solenoid(6);
	hatchclawsolenoidout = new frc::Solenoid(7);

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
