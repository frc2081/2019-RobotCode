/*
 * DriveManager.h
 *
 *  Created on: Jan 17, 2018
 *      Author: Matthew
 */

#ifndef SRC_DRIVESYSTEM_DRIVEMANAGER_H_
#define SRC_DRIVESYSTEM_DRIVEMANAGER_H_ 1
/* temp includes */
#include "SwerveLib.h"
#include "ControllerManager.h"
#include "IO.h"
#include "RobotCommands.h"
#include "frc/WPILib.h"
#include "GyroManager.h"

	class DriveManager {
	public:
		DriveManager(IO *io, RobotCommands *com, ControllerManager *cntls);

		void DriveManagerInit();
		void DriveManagerPeriodic();
		void DriveManagerAutoPeriodic();
		void DriveManagerDisabled();
		void UpdateDashboard();
		void ApplyFieldOrientedDrive();

		//Drive Base dimesions
		double drivebaseXDimesion = 29;
		double drivebaseYDimension = 17;

		//Default PID calibrations for swerve drive
		double turnP = -.03; 
		double turnI = 0;
		double turnD = 0;

		double drvP = 0;
		double drvI = 0;
		double drvD = 0;
		double drvF = 0;

	private:
		SwerveLib *_swervelib;
		IO *_io;
		RobotCommands *_commands;
		ControllerManager *_cntls;
		frc::PIDController *_lfdrvpid, *_rfdrvpid, *_lbdrvpid, *_rbdrvpid;
		frc::PIDController *_lfturnpid, *_rfturnpid, *_lbturnpid, *_rbturnpid;
		frc::Preferences *_prefs;
		double _drvpidi, _drvpidp, _drvpidd, _drvpidf;
		double _turnpidi, _turnpidp, _turnpidd;
		double _pidpollrate;
		double _maxdrivespeed; //Speed is in encoder pulses
		double _currangrf, _curranglf, _curranglb, _currangrb;

		double _lfwhlangoffset, _rfwhlangoffset, _lbwhlangoffset, _rbwhlangoffset;
		double _drvang, _drvmag, _drvrot;

		gyroManager *gyroManagerRun;
		double currentGyroReading = 0;

		bool swerveReset;

		double WhlAngCalcOffset(double, double);
		void ZeroEncoders();
		void CalculateVectors();
		void ApplyIntellegintSwerve();
		void AutoApplyPIDControl();
		void ApplyPIDControl();
		void UpdatePIDTunes();
	};







#endif /* SRC_DRIVESYSTEM_DRIVEMANAGER_H_ */
