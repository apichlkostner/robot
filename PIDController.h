/*
 * PIDController.h
 *
 *  Created on: 27.05.2017
 *      Author: Arthur Pichlkostner
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

namespace RobotDev {

class PIDController {
protected:
	float E_k;       // integrated error
	float e_k_l;     // last error
	// PID control parameter
	float Kp;
	float Ki;
	float Kd;

public:
	PIDController();
	PIDController(float Kp, float Ki, float Kd);
	virtual ~PIDController();

	float calc(float e_k, float dt);
};

} /* namespace RobotDev */

#endif
