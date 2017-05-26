/*
 * Motor.cpp
 *
 *  Created on: 26.05.2017
 *      Author: Arthur Pichlkostner
 */

#include "Motor.h"

namespace RobotDev {

Motor::Motor() {
	// TODO Auto-generated constructor stub

}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::setSpeeds(float v_l, float v_r)
{
	int pwm_min, pwm_max;

	int pwm_l = v_l * MAX_PWM;
	int pwm_r = v_r * MAX_PWM;

	// handle if pwms are too large
	if (pwm_l < pwm_r) {
		pwm_min = pwm_l;
		pwm_max = pwm_r;
	} else {
		pwm_min = pwm_r;
		pwm_max = pwm_l;
	}

	if (pwm_max > MAX_PWM) {
		pwm_r -= pwm_max - 255;
		pwm_l -= pwm_max - 255;
	}
	if (pwm_min < -MAX_PWM) {
		pwm_r -= pwm_min + 255;
		pwm_l -= pwm_min + 255;
	}

	leftDrive(pwm_l);
	rightDrive(pwm_r);
}

} /* namespace robot */
