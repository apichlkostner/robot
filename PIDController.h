/*
 * PIDController.h
 *
 *  Created on: 27.05.2017
 *      Author: arthur
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

namespace RobotDev {

class PIDController {
protected:
	float E_k;
	float e_k_l;
	const float Kp = 0.08;
	const float Ki = 0.01;
	const float Kd = 0.02;

public:
	PIDController();
	virtual ~PIDController();

	float calc(float e_k, float dt);
};

} /* namespace RobotDev */

#endif
