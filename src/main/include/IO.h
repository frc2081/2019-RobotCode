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
	void ioRobotPeriodic();

	int elevatorDesiredPos, elevatorActualPos;
	int elevatorPosTolerance = 0.25; //Distance from desired position to actual position for elevator to be considered at desired
	double elevatorMovePower = 1; //Motor power for elevator moves
	double elevatorMaxPosition = 33.75; //Max height elevator can move to, should be same as length of track
	double elevatorMinPosition = 0; //Min height elevator can move to

	//Scaling factors to convert encoder counts to physical distances
	const int sweerveanglogvoltagetodegrees = 72; //360 degrees travel per rev / 5 volts rev = 72 degrees per volt
	const double swerveencodercountstodistancecentimeters = .238; //Might be wrong or in inches instead of CM
	const double elevatorEncoderCountsToDistanceInches = .00048; //1024 counts/rev, 0.5 inch travel/rev; 0.5/1024 = .00048 inches per count
 	const double liftDistPerCountInches = .00306; //1024 counts/rev, 3.14 inches travel/rev; 3.14/1024 = .00306 inches per count

	//Swerve Drive Motors
	WPI_VictorSPX *drvlbmot;
	WPI_VictorSPX *drvrbmot;
	WPI_VictorSPX *drvlfmot;
	WPI_VictorSPX *drvrfmot;
	WPI_VictorSPX *turnlbmot;
	WPI_VictorSPX *turnrbmot;
	WPI_VictorSPX *turnlfmot;
	WPI_VictorSPX *turnrfmot;

	//Swerve Drive Encoders
	frc::Encoder *encdrvlb;
	frc::Encoder *encdrvlf;
	frc::Encoder *encdrvrb;
	frc::Encoder *encdrvrf;
	frc::AnalogPotentiometer *steerencdrvlb;
	frc::AnalogPotentiometer *steerencdrvlf;
	frc::AnalogPotentiometer *steerencdrvrb;
	frc::AnalogPotentiometer *steerencdrvrf;

	//Other Motors
	WPI_VictorSPX *ballintakemot;
	WPI_VictorSPX *liftdrivemot;
	WPI_VictorSPX *liftlbmot;
	WPI_VictorSPX *liftrbmot;
	WPI_VictorSPX *liftlfmot;
	WPI_VictorSPX *liftrfmot;
	WPI_VictorSPX *elevatormot;

	//Other Encoders
	frc::Encoder *liftlbenc;
	frc::Encoder *liftrbenc;
	frc::Encoder *liftlfenc;
	frc::Encoder *liftrfenc;
	frc::Encoder *elevatorenc;
	frc::Counter *liftdriveenc;

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

enum class HatchArmPos{
	HatchArmExtended,
	HatchArmRetracted
};

#endif /* SRC_IO_H_ */
