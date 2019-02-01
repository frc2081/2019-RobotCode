/*
 * IO.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#include "IO.h"

IO::IO() {

	//******************PWM assingments*****************
	//Swerve Drive Motors
	drvlbmot = new frc::Victor(3);
	drvrbmot  = new frc::Victor(2);
	drvlfmot  = new frc::Victor(1);
	drvrfmot  = new frc::Victor(0);
	turnlbmot = new frc::Victor(7);
	turnrbmot = new frc::Victor(6);
	turnlfmot = new frc::Victor(5);
	turnrfmot = new frc::Victor(4);

	//Other Motors
	ballintakemot = new VictorSPX(15);
	liftdrivemot = new VictorSPX(16);
	liftlbmot = new VictorSPX(17);
	liftrbmot = new VictorSPX(18);
	liftlfmot = new VictorSPX(19);
	liftrfmot = new VictorSPX(20);
	elevatormot = new VictorSPX(21); //fix port #
	
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

}

void IO::pollIO(){
}