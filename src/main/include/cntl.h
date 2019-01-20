/*
 * 	WPILib Controller library
 * 	cntl.h
 *
 * 	Organization: FRC2081
 *
 *  @author Lunar Dust
 *  @version 1.1 12/12/17
 *
 *  Do not redistribute this library without the permission of the author.
 */

#ifndef CNTL_H_
#define CNTL_H_

#include <frc/WPILib.h>
#include <math.h>

namespace cntl {
//For changing the mapping for different controllers
enum BTNENUMS {
	kbA = 1,
	kbB = 2,
	kbX = 3,
	kbY = 4,
	kbLB = 5,
	kbRB = 6,
	kbBack = 7,
	kbStart = 8,
	kbLS = 9,
	kbRS = 10
};

enum RANGELIMITS {
	kLX = 100,
	kLY = 100,
	kRX = 100,
	kRY = 100
};

class btn
{
	private:
		//JoystickButton object
		frc::JoystickButton*_raw;
		bool _state = false;
		bool _re = false;
		bool _held = false;

	public:
		//The current state of the given button
		bool State() const {return this->_state; }
		//True if the current state is true and the last state is false
		bool RE() const {return this->_re; }
		//The state of the button on the previous loop
		bool Held() const {return this->_held; }

		//Called for all buttons on a controller by calling UpdateCntl()
		void Update();

		//Button constructor
		btn(int, frc::Joystick**);
		virtual ~btn() = default;
};

class cntl
{
	private:
		//Joystick object
		frc::Joystick *_stick;

		double _deadzone;
		double _rangelimit;

	public:
		//Returns controller object, takes controller number and deadzone(0.01 to 1) as argument
		cntl(int, double, double);

		double LX;
		double LY;
		double RX;
		double RY;
		double RTrig;
		double LTrig;

		//Should be called for each controller at the beginning of each teleop loop
		void UpdateCntl();

		//Button objects of the controller
		btn *bA;
		btn *bB;
		btn *bX;
		btn *bY;
		btn *bLB;
		btn *bRB;
		btn *bBack;
		btn *bStart;
		btn *bLS;
		btn *bRS;
};
}

#endif /* CNTL_H_ */
