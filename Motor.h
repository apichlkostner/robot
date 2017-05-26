/*
 * Motor.h
 *
 *  Created on: 26.05.2017
 *      Author: arthur
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "RedBot.h"

namespace RobotDev {

class Motor : public RedBotMotors {
protected:
	const int MAX_PWM = 255;

public:
	Motor();
	virtual ~Motor();

	void setSpeeds(float v_l, float v_r);
};

} /* namespace robot */

#endif /* MOTOR_H_ */
