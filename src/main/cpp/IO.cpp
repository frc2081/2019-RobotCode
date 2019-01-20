/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"

IO::IO() {
	//Swerve Drive Motors
	drvlbmot = new frc::Victor(15);
	drvrbmot  = new frc::Victor(0);
	drvlfmot  = new frc::Victor(14);
	drvrfmot  = new frc::Victor(1);
	turnlbmot = new frc::Victor(10);
	turnrbmot = new frc::Victor(5);
	turnlfmot = new frc::Victor(9);
	turnrfmot = new frc::Victor(6);

	//Swerve Drive Encoders
	encdrvlb = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
	encdrvlf = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);
	encdrvrb = new frc::Encoder(4, 5, false, frc::Encoder::EncodingType::k4X);
	encdrvrf= new frc::Encoder(6, 7, false, frc::Encoder::EncodingType::k4X);
	steerencdrvlb = new frc::AnalogPotentiometer(1,360,0);
	steerencdrvlf = new frc::AnalogPotentiometer(0,360,0);
	steerencdrvrb = new frc::AnalogPotentiometer(3,360,0);
	steerencdrvrf = new frc::AnalogPotentiometer(2,360,0);

	encdrvlb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvrb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvlf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	encdrvrf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);

	swerveresetone = new frc::DigitalInput(0);
	swerveresettwo = new frc::DigitalInput(1);
}

void IO::pollIO(){
	frc::SmartDashboard::PutNumber("Right Front Steer Encoder Pos", steerencdrvrf->Get());
	frc::SmartDashboard::PutNumber("Left Front Steer Encoder Pos", steerencdrvlf->Get());
	frc::SmartDashboard::PutNumber("Right Back Steer Encoder Pos", steerencdrvrb->Get());
	frc::SmartDashboard::PutNumber("Left Back Steer Encoder Pos", steerencdrvlb->Get());

	frc::SmartDashboard::PutNumber("Right Front Drive Encoder Pos", steerencdrvrf->Get());
	frc::SmartDashboard::PutNumber("Left Front Drive Encoder Pos", steerencdrvlf->Get());
	frc::SmartDashboard::PutNumber("Right Back Drive Encoder Pos", steerencdrvrb->Get());
	frc::SmartDashboard::PutNumber("Left Back Drive Encoder Pos", steerencdrvlb->Get());

	frc::SmartDashboard::PutBoolean("Swerve Reset Command 1", swerveresetone->Get());
	frc::SmartDashboard::PutBoolean("Swerve Reset Command 2", swerveresettwo->Get());
}