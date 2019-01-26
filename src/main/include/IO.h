/*
 * IO.h
 *
 *  Created on: Jan 17, 2018
 *      Author: 1800855
 */

#ifndef SRC_IO_H_
#define SRC_IO_H_ 1


#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

/*
 * Contains an instance of all sensors, motors, and actuators to be passed
 * throughout the entire program
 */
class IO {
public:

	IO();
	void pollIO();

	// The scaling to multiply the voltage by to get a meaningful unit 360 degrees / 5 volts = 72
	const int sweerveanglogvoltagetodegrees = 72;
	const double swerveencodercountstodistancecentimeters = .238;
	
	//Swerve Drive Motors
	frc::Victor *drvlbmot;
	frc::Victor *drvrbmot;
	frc::Victor *drvlfmot;
	frc::Victor *drvrfmot;
	frc::Victor *turnlbmot;
	frc::Victor *turnrbmot;
	frc::Victor *turnlfmot;
	frc::Victor *turnrfmot;

	//Swerve Drive Encoders
	frc::Encoder *encdrvlb;
	frc::Encoder *encdrvlf;
	frc::Encoder *encdrvrb;
	frc::Encoder *encdrvrf;
	frc::AnalogPotentiometer *steerencdrvlb;
	frc::AnalogPotentiometer *steerencdrvlf;
	frc::AnalogPotentiometer *steerencdrvrb;
	frc::AnalogPotentiometer *steerencdrvrf;

	frc::DigitalInput *swerveresetone;
	frc::DigitalInput *swerveresettwo;

	//Other Motors
	VictorSPX *ballintakemot;
	VictorSPX *liftdrivemot;
	VictorSPX *drivelbmot;
	VictorSPX *driverbmot;
	VictorSPX *drivelfmot;
	VictorSPX *driverfmot;
	VictorSPX *elevatormot;
	

	//Other Encoders
	frc::Encoder *liftdriveenc;
	frc::Encoder *liftlbenc;
	frc::Encoder *liftrbenc;
	frc::Encoder *liftlfenc;
	frc::Encoder *liftrfenc;
	frc::Encoder *elevatorenc;

	frc::Compressor *compressor;

	//Solenoids
	frc::Solenoid *ballarmsolenoidin;
	frc::Solenoid *hatcharmsolenoidin;
	frc::Solenoid *ballshootersolenoidin;
	frc::Solenoid *hatchclawsolenoidin;
	frc::Solenoid *ballarmsolenoidout;
	frc::Solenoid *hatcharmsolenoidout;
	frc::Solenoid *ballshootersolenoidout;
	frc::Solenoid *hatchclawsolenoidout;
	






	double armP = .7;
	double armI = 0;
	double armD = 0;

private:

};

#endif /* SRC_IO_H_ */
