/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"

IO::IO() {
	//Swerve Drive Motors
	drvlbmot = new frc::Victor(4);
	drvrbmot  = new frc::Victor(7);
	drvlfmot  = new frc::Victor(9);
	drvrfmot  = new frc::Victor(13);
	turnlbmot = new frc::Victor(3);
	turnrbmot = new frc::Victor(6);
	turnlfmot = new frc::Victor(8);
	turnrfmot = new frc::Victor(12);

	//Swerve Drive Encoders
	encdrvlb = new frc::Encoder(16, 17, false, frc::Encoder::EncodingType::k4X);
	encdrvlf = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);
	encdrvrb = new frc::Encoder(4, 5, false, frc::Encoder::EncodingType::k4X);
	encdrvrf= new frc::Encoder(6, 7, false, frc::Encoder::EncodingType::k4X);
	steerencdrvlb = new frc::AnalogPotentiometer(1,360,0);
	steerencdrvlf = new frc::AnalogPotentiometer(0,360,0);
	steerencdrvrb = new frc::AnalogPotentiometer(3,360,0);
	steerencdrvrf = new frc::AnalogPotentiometer(2,360,0);

	//encdrvlb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvrb->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvlf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	//encdrvrf->SetDistancePerPulse(swerveencodercountstodistancecentimeters);
	swerveresetone = new frc::DigitalInput(0);
	swerveresettwo = new frc::DigitalInput(1);
}

void IO::pollIO(){
}