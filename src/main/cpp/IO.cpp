/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"
#include "ctre/Phoenix.h"

IO::IO() {
	PDP = new frc::PowerDistributionPanel(20);
	PCM = new frc::Compressor(0);

	//Swerve Drive Motors
	drvrfmot = new WPI_VictorSPX(0);		
	drvlfmot = new WPI_VictorSPX(1);
	drvrbmot = new WPI_VictorSPX(2);
	drvlbmot = new WPI_VictorSPX(3);
	turnrfmot = new WPI_VictorSPX(4);
	turnlfmot = new WPI_VictorSPX(5);
	turnrbmot = new WPI_VictorSPX(6);
	turnlbmot = new WPI_VictorSPX(7);
	//Other Motors
	liftdrivemot = new WPI_VictorSPX(12);	
	elevatormot = new WPI_VictorSPX(13);

	liftrfmot = new WPI_VictorSPX(8);	
	liftlfmot = new WPI_VictorSPX(9);
	liftrbmot = new WPI_VictorSPX(10);
	liftlbmot = new WPI_VictorSPX(11);
	//PWM 14-18 used as DIO inputs
	ballintakemot = new WPI_VictorSPX(14);

	//Swerve Drive Encoders
	encdrvrf = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
	encdrvlf = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);
	encdrvrb = new frc::Encoder(4, 5, false, frc::Encoder::EncodingType::k4X);
	encdrvlb = new frc::Encoder(6, 7, false, frc::Encoder::EncodingType::k4X);	
	//DIO 10-13 are used as PWM outputs for climb motors		
	steerencdrvrf = new frc::AnalogPotentiometer(0,360,0);	
	steerencdrvlf = new frc::AnalogPotentiometer(1,360,0);	
	steerencdrvrb = new frc::AnalogPotentiometer(2,360,0);	
	steerencdrvlb = new frc::AnalogPotentiometer(3,360,0);
	//Other Encoders and DIO
	elevatorenc = new frc::Encoder(10, 11);	
	liftrfenc = new frc::Encoder(14 ,15);
	liftlfenc = new frc::Encoder(16, 17);
	liftrbenc = new frc::Encoder(18, 19);
	liftlbenc = new frc::Encoder(20, 21);	
	liftdriveenc = new frc::Counter(22);
	hatchDetectorOne = new frc::DigitalInput(8);
	hatchDetectorTwo = new frc::DigitalInput(9);

		
	//Solenoids
	//compressor = new frc::Compressor();
	hatcharmsolenoidout = new frc::Solenoid(0);	
	hatcharmsolenoidin = new frc::Solenoid(1);	
	hatchclawsolenoidin = new frc::Solenoid(2);	
	hatchclawsolenoidout = new frc::Solenoid(3);	
	ballarmsolenoidin = new frc::Solenoid(4);
	ballarmsolenoidout = new frc::Solenoid(5);
	framestandsolenoidin = new frc::Solenoid(6);
	framestandsolenoidout = new frc::Solenoid(7);

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

	frc::SmartDashboard::PutNumber("Elevator Move Power", elevatorMovePower);
	frc::SmartDashboard::PutNumber("Elevator Pos Tolerance", elevatorPosTolerance);
	frc::SmartDashboard::PutNumber("econtrolMode", 0
	);
}

void IO::ioPeriodic(){
	elevatorActualPos = elevatorenc->GetDistance();


	//Move elevator up and down until desired position is reached
	double elevatorMotorPower = 0;
	if(elevatorActualPos < elevatorDesiredPos - elevatorPosTolerance){
		elevatorMotorPower = elevatorMovePower;
		frc::SmartDashboard::PutNumber("econtrolMode", 2);
	} else if (elevatorActualPos > elevatorDesiredPos + elevatorPosTolerance){
		elevatorMotorPower = -elevatorMovePower;
		frc::SmartDashboard::PutNumber("econtrolMode", 3);
	} else { 
		elevatorMotorPower = 0;
		frc::SmartDashboard::PutNumber("econtrolMode", 4);
	}	
	
	//Stop elevator if above or below position limits and trying to move even further past them
	if(elevatorActualPos < elevatorMinPosition && elevatorMotorPower < 0){
		elevatorMotorPower = 0;
		frc::SmartDashboard::PutNumber("econtrolMode", 0);
	}else if(elevatorActualPos > elevatorMaxPosition && elevatorMotorPower > 0){
		elevatorMotorPower = 0;
		frc::SmartDashboard::PutNumber("econtrolMode", 1);
	}

	//Soft landing code - slow down the elevator just before it hits the bottom hard stop
	if(elevatorMotorPower < 0 && elevatorActualPos < elevatorSoftLandingPosition) {elevatorMotorPower = elevatorLandingPower;}

	elevatormot->Set(elevatorMotorPower);
}

void IO::ioRobotPeriodic(){
	double emotpower = elevatormot->Get();
	frc::SmartDashboard::PutNumber("Elevator Des Pos", elevatorDesiredPos);
	frc::SmartDashboard::PutNumber("Elevator Act Pos", elevatorenc->GetDistance());
	frc::SmartDashboard::PutNumber("Elevator Motor Power", emotpower);
	elevatorMovePower = frc::SmartDashboard::GetNumber("Elevator Move Power", elevatorMovePower);
	elevatorPosTolerance = frc::SmartDashboard::GetNumber("Elevator Pos Tolerance", elevatorPosTolerance);

}
