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
	void ioPeriodic();
	void robotMechanismPeriodic();

	int elevatorDesiredPos, elevatorActualPos;
	int elevatorPosTolerance = 0.5;
	double elevatorMovePower = 1;

	// The scaling to multiply the voltage by to get a meaningful unit 360 degrees / 5 volts = 72
	const int sweerveanglogvoltagetodegrees = 72;
	const double swerveencodercountstodistancecentimeters = .238;
	
	const double elevatorEncoderCountsToDistanceInches = .00097;

	//Swerve Drive Motors
	frc::VictorSP *drvlbmot;
	frc::VictorSP *drvrbmot;
	frc::VictorSP *drvlfmot;
	frc::VictorSP *drvrfmot;
	frc::VictorSP *turnlbmot;
	frc::VictorSP *turnrbmot;
	frc::VictorSP *turnlfmot;
	frc::VictorSP *turnrfmot;

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
	frc::VictorSP *ballintakemot;
	frc::VictorSP *liftdrivemot;
	frc::VictorSP *liftlbmot;
	frc::VictorSP *liftrbmot;
	frc::VictorSP *liftlfmot;
	frc::VictorSP *liftrfmot;
	frc::VictorSP *elevatormot;

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
 	double liftDistPerCountInches;

private:

};

enum class HatchArmPos{
	HatchArmExtended,
	HatchArmRetracted
};

#endif /* SRC_IO_H_ */
