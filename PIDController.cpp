/*
 * PIDController.cpp
 *
 *  Created on: 27.05.2017
 *      Author: arthur
 */

#include "PIDController.h"

namespace RobotDev {

PIDController::PIDController() {
	// TODO Auto-generated constructor stub

}

PIDController::~PIDController() {
	// TODO Auto-generated destructor stub
}

float PIDController::calc(float e_k, float dt)
{
	float e_P = e_k;
	float e_I = E_k + e_k * dt;
	float e_D = (e_k - e_k_l) / dt;
	float w = Kp * e_P + Ki * e_I + Kd * e_D;

	e_k_l = e_k;
	E_k = e_I;

	return w;
}

} /* namespace RobotDev */
