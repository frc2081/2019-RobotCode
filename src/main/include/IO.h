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
#include "rev/CANSparkMax.h"

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

	double elevatorDesiredPos, elevatorActualPos;
	double elevatorPosTolerance = 0.3; //Distance from desired position to actual position for elevator to be considered at desired
	double elevatorMovePower = 1; //Motor power for elevator moves
	double elevatorMaxPosition = 27.5; //Max height elevator can move to, should be same as length of track
	double elevatorMinPosition = 0; //Min height elevator can move to
	double elevatorSoftLandingPosition = 1.5;
	double elevatorLandingPower = -.4;

	//Scaling factors to convert encoder counts to physical distances
	const int sweerveanglogvoltagetodegrees = 72; //360 degrees travel per rev / 5 volts rev = 72 degrees per volt
	const double swerveencodercountstodistancecentimeters = .238; //Might be wrong or in inches instead of CM
	const double elevatorEncoderCountsToDistanceInches = .00048; //1024 counts/rev, 0.5 inch travel/rev; 0.5/1024 = .00048 inches per count
 	const double liftDistPerCountInches = .00306; //1024 counts/rev, 3.14 inches travel/rev; 3.14/1024 = .00306 inches per count
	const double ballIntakeCountsPerRevolution = .000976; //1024 counts/rev, 1 rev / 1024 = .000976

	frc::PowerDistributionPanel *PDP;
	frc::Compressor *PCM;

	//Swerve Drive Motors
	rev::CANSparkMax *drvlbmot;
	rev::CANSparkMax *drvrbmot;
	rev::CANSparkMax *drvlfmot;
	rev::CANSparkMax *drvrfmot;
	WPI_VictorSPX *turnlbmot;
	WPI_VictorSPX *turnrbmot;
	WPI_VictorSPX *turnlfmot;
	WPI_VictorSPX *turnrfmot;

	//Swerve Drive Encoders
	rev::CANEncoder *drvlbenc;
	rev::CANEncoder *drvrbenc;
	rev::CANEncoder *drvlfenc;
	rev::CANEncoder *drvrfenc;
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
	frc::Encoder *ballintakeenc;

	//Solenoids
	frc::Solenoid *ballarmsolenoidin;
	frc::Solenoid *ballarmsolenoidout;	
	frc::Solenoid *hatcharmsolenoidin;
	frc::Solenoid *hatcharmsolenoidout;
	frc::Solenoid *hatchclawsolenoidin;
	frc::Solenoid *hatchclawsolenoidout;
	frc::Solenoid *framestandsolenoidin;	
	frc::Solenoid *framestandsolenoidout;


	//Other Inputs
	frc::DigitalInput *hatchDetectorOne;
	frc::DigitalInput *hatchDetectorTwo;

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
