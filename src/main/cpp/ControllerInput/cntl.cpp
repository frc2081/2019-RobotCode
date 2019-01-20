/*
 * 	WPILib Controller library
 * 	cntl.cpp
 *
 * 	Organization: FRC2081
 *
 *  @author Lunar Dust
 *  @version 1.1 12/12/17
 *
 *  Do not redistribute this library without the permission of the author.
 */

#include "cntl.h"
namespace cntl {

btn::btn(int button, frc::Joystick **s) {
	_raw = new frc::JoystickButton(*s, button);
}

void btn::Update() {
	_held = _state;
	_state = _raw->Grab();
	_re = false;
	if(!_held && _held != _state) {
		_re = true;
	}
}

cntl::cntl(int port, double deadzone, double rangelimit) {
	_stick = new frc::Joystick(port);
	RX = RY = 0.0;
	LX = LY = 0.0;
	RTrig = LTrig = 0.0;

	_deadzone = deadzone;
	_rangelimit = rangelimit;

	bA = new btn(kbA, &_stick);
	bB = new btn(kbB, &_stick);
	bX = new btn(kbX, &_stick);
	bY = new btn(kbY, &_stick);
	bLB = new btn(kbLB, &_stick);
	bRB = new btn(kbRB, &_stick);
	bBack = new btn(kbBack, &_stick);
	bStart = new btn(kbStart, &_stick);
	bLS = new btn(kbLS, &_stick);
	bRS = new btn(kbRS, &_stick);
}

void ApplyDeadzone(double *joystick, double input, double zone) {
	double o = fabs(input) - zone;
	o /= (1 - zone);
	if(o < 0) {
		o = 0;
	}
	if(input < 0) {
		o *= -1;
	}

	// Converts -0.0 to 0.0
	/*if((o + 0.0) == 0.0) {
		o = 0.0;
	}*/
	o += 0.0; // Faster version, same effect

	*joystick = o;
}

void ApplyLimit(double *joystick, int limit)
{
	// This might be wrong
	double o = (double)limit / 100.0;
	if(*joystick > o)
		*joystick = o;
}

void cntl::UpdateCntl() {
	ApplyDeadzone(&LX, _stick->GetX(), _deadzone);
	ApplyDeadzone(&LY, _stick->GetY(), _deadzone);
	ApplyDeadzone(&RX, _stick->GetRawAxis(4), _deadzone);
	ApplyDeadzone(&RY, _stick->GetRawAxis(5), _deadzone);
	ApplyDeadzone(&RTrig, _stick->GetRawAxis(6), _deadzone);
	ApplyDeadzone(&LTrig, _stick->GetRawAxis(7), _deadzone);

	ApplyLimit(&LX, kLX);
	ApplyLimit(&LY, kLY);
	ApplyLimit(&RX, kRX);
	ApplyLimit(&RY, kRY);

	RY *= -1;
	LY *= -1;

	bA->Update();
	bB->Update();
	bX->Update();
	bY->Update();
	bLB->Update();
	bRB->Update();
	bBack->Update();
	bStart->Update();
	bLS->Update();
	bRS->Update();
}
}
