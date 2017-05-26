/*
 * robot.h
 *
 *  Created on: 26.05.2017
 *      Author: arthur
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include "RedBot.h"
#include "Motor.h"

namespace RobotDev {

struct vector {
	float x;
	float y;
};

struct vectors {
	struct vector *v;
	int i;
	int size;
};

class Controller {
public:
	float E_k;
	float e_k_l;
	const float Kp = 0.08;
	const float Ki = 0.01;
	const float Kd = 0.02;
};

class Robot {
private:
	const int countsPerRev = 192;
	const float R = 0.032;
	const float L = 0.165;

	int lCount;
	int rCount;

	float v_l;
	float v_r;

	float pos[2];
	float theta;

	float goal[2];

	Controller cntrl;
	Motor motor;
	RedBotEncoder encoder = RedBotEncoder(A2, 10);

	float v_s;
	float v_r_s;
	float v_l_s;
	float w;
	float v_x;
	float v_y;
	float omega;

	boolean started;

	struct vectors route;

public:
	Robot();
	virtual ~Robot();

	void sense(float dt);
	void updateState(float dt);
	void control(float dt);
	void actuate(float dt);

	void setRoute(struct vectors v);
	void start(void);
	void stop(void);

	void printDebug(void);
};

} /* namespace robot */

#endif /* ROBOT_H_ */
