/*
* DriveManager.cpp
*
*  Created on: Jan 17, 2018
*      Author: Matthew
*/
#include "DriveManager.h"

DriveManager::DriveManager(IO *io, RobotCommands *com, ControllerManager *cntls) {
	_io = io;
	_commands = com;
	_swervelib = new SwerveLib(drivebaseXDimesion, drivebaseYDimension);
	_cntls = cntls;

	_drvpidi = drvI;
	_drvpidd = drvD;
	_drvpidp = drvP;
	_drvpidf = drvF;
	_turnpidp = turnP;
	_turnpidi = turnI;
	_turnpidd = turnD;

	_pidpollrate = 0.01;

	_drvang = 0;
	_drvmag = 0;
	_drvrot = 0;

	/* Max Speed
	5676 RPM on NEO brushless motor	
	Swerve gear ratio = 6.67:1
	5676 / 6.67 = 851 max RPM of the swerve wheel
	4" wheel diameter = 12.56" circumference
	851 RPM / 60 seconds/minute * 12.56 inches/rev = 178 inches/sec
	178 in/s = 14.83 ft/s
	*/
	_maxdrivespeed = 1; //Speed is in encoder pulses

	_currangrf = 0;
	_curranglf = 0;
	_curranglb = 0;
	_currangrb = 0;
	_lfwhlangoffset = 0;
	_rfwhlangoffset = 0;
	_lbwhlangoffset = 0;
	_rbwhlangoffset = 0;

	swerveReset = false;

	//Set up swerve turning motor PID controllers
	_lfturnpid = new frc::PIDController(_turnpidp, _turnpidi, _turnpidp, io->steerencdrvlf, io->turnlfmot, _pidpollrate);
	_rfturnpid = new frc::PIDController(_turnpidp, _turnpidi, _turnpidp, io->steerencdrvrf, io->turnrfmot, _pidpollrate);
	_rbturnpid = new frc::PIDController(_turnpidp, _turnpidi, _turnpidp, io->steerencdrvrb, io->turnrbmot, _pidpollrate);
	_lbturnpid = new frc::PIDController(_turnpidp, _turnpidi, _turnpidp, io->steerencdrvlb, io->turnlbmot, _pidpollrate);
	_lfturnpid->SetInputRange(0, 360);
	_lfturnpid->SetOutputRange(-1, 1);
	_lfturnpid->SetContinuous();
	_lfturnpid->Enable();
	_rfturnpid->SetInputRange(0, 360);
	_rfturnpid->SetOutputRange(-1, 1);
	_rfturnpid->SetContinuous();
	_rfturnpid->Enable();
	_lbturnpid->SetInputRange(0, 360);
	_lbturnpid->SetOutputRange(-1, 1);
	_lbturnpid->SetContinuous();
	_lbturnpid->Enable();
	_rbturnpid->SetInputRange(0, 360);
	_rbturnpid->SetOutputRange(-1, 1);
	_rbturnpid->SetContinuous();
	_rbturnpid->Enable();

	//Preferences file to save swerve drive encoder offset calibrations
	_prefs = frc::Preferences::GetInstance();
	_lfwhlangoffset = _prefs->GetDouble("LFOffset", 0);
	_rfwhlangoffset = _prefs->GetDouble("RFOffset", 0);
	_lbwhlangoffset = _prefs->GetDouble("LBOffset", 0);
	_rbwhlangoffset = _prefs->GetDouble("RBOffset", 0);


	//_lfdrvpid = new rev::CANPIDController(*_io->drvlfmot);
	//_rfdrvpid = _io->drvrfmot->GetPIDController();
	//_lbdrvpid = _io->drvlbmot->GetPIDController();
	//_rbdrvpid = _io->drvrbmot->GetPIDController();
}

void DriveManager::DriveManagerInit() {

	frc::SmartDashboard::PutNumber("Swerve Turn P", turnP);
	frc::SmartDashboard::PutNumber("Swerve Turn I", turnI);
	frc::SmartDashboard::PutNumber("Swerve Turn D", turnD);

	frc::SmartDashboard::PutNumber("Swerve Drive P", drvP);
	frc::SmartDashboard::PutNumber("Swerve Drive I", drvI);
	frc::SmartDashboard::PutNumber("Swerve Drive D", drvD);
	frc::SmartDashboard::PutNumber("Swerve Drive F", drvF);

	frc::SmartDashboard::PutBoolean("Swerve Reset", swerveReset);
}

void DriveManager::DriveManagerPeriodic() {
	UpdatePIDTunes();
	CalculateVectors();
	ApplyIntellegintSwerve();
	ApplyPIDControl();
}

void DriveManager::DriveManagerAutoPeriodic() {
	CalculateVectors();
	ApplyIntellegintSwerve();
	AutoApplyPIDControl();
}

void DriveManager::DriveManagerDisabled(){
	if (swerveReset)  {
	ZeroEncoders();
  	}
}

void DriveManager::ZeroEncoders() {
	_lfwhlangoffset = _io->steerencdrvlf->Get();
	_rfwhlangoffset = _io->steerencdrvrf->Get();
	_lbwhlangoffset = _io->steerencdrvlb->Get();
	_rbwhlangoffset = _io->steerencdrvrb->Get();
	_prefs->PutDouble("LFOffset", _lfwhlangoffset);
	_prefs->PutDouble("RFOffset", _rfwhlangoffset);
	_prefs->PutDouble("LBOffset", _lbwhlangoffset);
	_prefs->PutDouble("RBOffset", _rbwhlangoffset);

	printf("\n\n");
	printf("***********************************************\n");
	printf("**Swerve Steering Encoder Offsets Calibrated!**\n");
	printf("***********************************************\n");
	printf("New LF offset: %.2f  ", _lfwhlangoffset);
	printf("New RF offset: %.2f  ", _rfwhlangoffset);
	printf("New LB offset: %.2f  ", _lbwhlangoffset);
	printf("New RB offset: %.2f  ", _rbwhlangoffset);
	printf("***********************************************\n");
	printf("***********************************************\n");
	printf("\n\n");
}

//Function to apply the wheel angle encoder calibration offsets
//Rather than exactly line each wheel encoder up so that the 0 position of all four is the same
//This allows them to be installed at any angle realtive to each other
//The robot wheels are then all lined up and the position of each encoder is recorded
//and applied as a offset to the final commanded angle
double DriveManager::WhlAngCalcOffset(double command, double offset) {
	double target = command + offset;
	if (target > 360) target -= 360;
	return target;
}

//Calculate swerve drive wheel vectors
//Only calculate new vectors if current command is non-zero.
//Otherwise, keep the current wheel angles and set speeds to 0
//This prevents the swerve wheels from always returning to "0" angle when there is no command from the driver
//Doing so makes for much smoother starts and stops
void DriveManager::CalculateVectors() {

	//Determine if the driver or the guidance system is in control of the drivetrain
	//and use the appropriate drive commands
	_drvangprevious = _drvang;
	if(_commands->guidanceSysActive == true){
		_drvang = _commands->autodrvang;
		_drvmag = _commands->autodrvmag;
		_drvrot = _commands->autodrvrot;
	} else {
		 if(fabs(_commands->drvmag) < 0.05) _drvang = _drvangprevious;
		 else _drvang = _commands->drvang;

		_drvmag = pow(_commands->drvmag, 2);
		
		if(fabs(_commands->drvrot) < 0.05) _drvrot = 0;
		else _drvrot = _commands->drvrot;

		if(_drvrot > 0) _drvrot = pow(_drvrot, 2);
		else _drvrot = -pow(_drvrot, 2);
		//if(_commands->fieldOrientedDrive == true) ApplyFieldOrientedDrive();
	}

	//prevent the swerve from reorienting the wheels if the drive command is zero
	//otherwise the wheels will snap to a new vector while the robot is still
	//moving and possibly tip the robot	
	_currangrf = _swervelib->whl->angleRF;
	_curranglf = _swervelib->whl->angleLF;
	_currangrb = _swervelib->whl->angleRB;
	_curranglb = _swervelib->whl->angleLB;

	//TODO: Don't use an equivalency comparator with doubles!
	if (_drvmag != 0 || _drvrot != 0) {
		_swervelib->CalcWheelVect(_drvmag, _drvang, _drvrot);
	} else {
		_swervelib->whl->speedLF = 0;
		_swervelib->whl->speedRF = 0;
		_swervelib->whl->speedLB = 0;
		_swervelib->whl->speedRB = 0;

		_swervelib->whl->angleRF = _currangrf;
		_swervelib->whl->angleLF = _curranglf;
		_swervelib->whl->angleRB = _currangrb;
		_swervelib->whl->angleLB = _curranglb;
	}
}

//This function modifies the output of the swerve library to control the turn motors more intelligently
//It works to prevent the wheels from turning completely around when they would only need to move a bit and then reverse to reach a target vector
void DriveManager::ApplyIntellegintSwerve() {
	if (fabs(_swervelib->whl->angleRF - _currangrf) > 90 &&
			(_swervelib->whl->angleRF - _currangrf < 270)) {
		_swervelib->whl->angleRF = ((int)_swervelib->whl->angleRF + 180) % 360;
		_swervelib->whl->speedRF *= -1;
	}
	if (fabs(_swervelib->whl->angleLF - _curranglf) > 90 &&
			(_swervelib->whl->angleLF - _curranglf < 270)) {
		_swervelib->whl->angleLF = ((int)_swervelib->whl->angleLF + 180) % 360;
		_swervelib->whl->speedLF *= -1;
	}
	if (fabs(_swervelib->whl->angleRB - _currangrb) > 90 &&
			(_swervelib->whl->angleRB - _currangrb < 270)) {
		_swervelib->whl->angleRB = ((int)_swervelib->whl->angleRB + 180) % 360;
		_swervelib->whl->speedRB *= -1;
	}
	if (fabs(_swervelib->whl->angleLB - _curranglb) > 90 &&
			(_swervelib->whl->angleLB - _curranglb < 270)) {
		_swervelib->whl->angleLB = ((int)_swervelib->whl->angleLB + 180) % 360;
		_swervelib->whl->speedLB *= -1;
	}
}

//Function to prevent swerve drive from moving if the wheels have not yet moved to the target angle
//Mainly used for autonomous navigation and following paths that have sharp corners
void CheckSetpoint(frc::PIDController *pid, double setpoint, double encoder, double speed) {
	/*
	 * if encoder value within 10 degrees of setpoint &&
	 * if setpoint near 0 -> detect values near 360
	 * set speed setpoint to speed
	 * else speed setpoint to 0
	 */
	if  (encoder < setpoint + 10 && encoder > setpoint - 10) {
		pid->SetSetpoint(speed);
	} else if ((360 - encoder) < setpoint + 10 && (360 - encoder) > setpoint + 10) {
		pid->SetSetpoint(speed);
	} else pid->SetSetpoint(0);
}

void DriveManager::AutoApplyPIDControl() {}

void DriveManager::ApplyPIDControl() {

	_lfturnpid->SetSetpoint(WhlAngCalcOffset(_swervelib->whl->angleLF, _lfwhlangoffset));
	_rfturnpid->SetSetpoint(WhlAngCalcOffset(_swervelib->whl->angleRF, _rfwhlangoffset));
	_lbturnpid->SetSetpoint(WhlAngCalcOffset(_swervelib->whl->angleLB, _lbwhlangoffset));
	_rbturnpid->SetSetpoint(WhlAngCalcOffset(_swervelib->whl->angleRB, _rbwhlangoffset));

	_swervelib->whl->speedLF *= _maxdrivespeed;
	_swervelib->whl->speedRF *= _maxdrivespeed;
	_swervelib->whl->speedLB *= _maxdrivespeed;
	_swervelib->whl->speedRB *= _maxdrivespeed;

	_io->drvlfmot->Set(_swervelib->whl->speedLF);
	_io->drvrfmot->Set(_swervelib->whl->speedRF);
	_io->drvlbmot->Set(_swervelib->whl->speedLB);
	_io->drvrbmot->Set(_swervelib->whl->speedRB);

	//_lfdrvpid->SetSetpoint(_swervelib->whl->speedLF);
	//_rfdrvpid->SetSetpoint(_swervelib->whl->speedRF);
	//_lbdrvpid->SetSetpoint(_swervelib->whl->speedLB);
	//_rbdrvpid->SetSetpoint(_swervelib->whl->speedRB);
}

void DriveManager::UpdateDashboard(){

	//Swerve Desired Wheel Vectors
	frc::SmartDashboard::PutNumber("Swerve Left Front Angle Desired", _swervelib->whl->angleLF);	
	frc::SmartDashboard::PutNumber("Swerve Right Front Angle Desired", _swervelib->whl->angleRF);	
	frc::SmartDashboard::PutNumber("Swerve Left Back Angle Desired", _swervelib->whl->angleLB);	
	frc::SmartDashboard::PutNumber("Swerve Right Back Angle Desired", _swervelib->whl->angleRB);	

	frc::SmartDashboard::PutNumber("Swerve Left Front Speed Desired", _swervelib->whl->speedLF);	
	frc::SmartDashboard::PutNumber("Swerve Right Front Speed Desired", _swervelib->whl->speedRF);	
	frc::SmartDashboard::PutNumber("Swerve Left Back Speed Desired", _swervelib->whl->speedLB);	
	frc::SmartDashboard::PutNumber("Swerve Right Back Speed Desired", _swervelib->whl->speedRB);

	//Swerve Actual Wheel Vectors
	frc::SmartDashboard::PutNumber("Swerve Left Front Angle Actual", _io->steerencdrvlf->Get());
	frc::SmartDashboard::PutNumber("Swerve Right Front Angle Actual", _io->steerencdrvrf->Get());	
	frc::SmartDashboard::PutNumber("Swerve Left Back Angle Actual", _io->steerencdrvlb->Get());
	frc::SmartDashboard::PutNumber("Swerve Right Back Angle Actual", _io->steerencdrvrb->Get());

	//Swerve Encoder offset calibrations
	frc::SmartDashboard::PutNumber("Swerve Left Front Encoder Offset", _lfwhlangoffset);
	frc::SmartDashboard::PutNumber("Swerve Right Front Encoder Offset", _rfwhlangoffset);
	frc::SmartDashboard::PutNumber("Swerve Left Back Encoder Offset", _lbwhlangoffset);
	frc::SmartDashboard::PutNumber("Swerve Right Back Encoder Offset", _rbwhlangoffset);

	swerveReset = frc::SmartDashboard::GetBoolean("Swerve Reset", swerveReset);
}

void DriveManager::UpdatePIDTunes(){

	_turnpidp = frc::SmartDashboard::GetNumber("Swerve Turn P", turnP);
	_turnpidi = frc::SmartDashboard::GetNumber("Swerve Turn I", turnI);
	_turnpidd = frc::SmartDashboard::GetNumber("Swerve Turn D", turnD);

	_lfturnpid->SetP(_turnpidp);
	_lfturnpid->SetI(_turnpidi);
	_lfturnpid->SetD(_turnpidd);
	
	_rfturnpid->SetP(_turnpidp);
	_rfturnpid->SetI(_turnpidi);
	_rfturnpid->SetD(_turnpidd);

	_lbturnpid->SetP(_turnpidp);
	_lbturnpid->SetI(_turnpidi);
	_lbturnpid->SetD(_turnpidd);

	_rbturnpid->SetP(_turnpidp);
	_rbturnpid->SetI(_turnpidi);
	_rbturnpid->SetD(_turnpidd);
}

void DriveManager::ApplyFieldOrientedDrive(){
	currentGyroReading = gyroManagerRun->getLastValue() - 90;
	_drvang = _drvang - currentGyroReading;
	while(_drvang < 0) _drvang += 360;
	while(_drvang > 360) _drvang -= 360;
}