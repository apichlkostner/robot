/*
 * robot.h
 *
 *  Created on: 26.05.2017
 *      Author: Arthur Pichlkostner
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include "RedBot.h"
#include "Motor.h"
#include "RobotSound.h"
#include "PIDController.h"
#include "DistanceSensor.h"

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


class Robot {
private:
	const int countsPerRev = 192;
	const float R = 0.032;
	const float L = 0.165;
	const float stopDistanceSquared = 0.01;
	const int bumpThreshold = 70;

	int lCount;
	int rCount;

	float v_l;
	float v_r;

	vector pos;
	float theta;

	PIDController cntrl;
	RobotSound sound;
	Motor motor;
	DistanceSensor d_sensors[2] = {DistanceSensor(A0), DistanceSensor(A1)};
	RedBotEncoder encoder = RedBotEncoder(A2, 10);
	RedBotBumper lBumper = RedBotBumper(3);
	RedBotBumper rBumper = RedBotBumper(11);
	RedBotAccel accel;


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
	void checkState(float dt);
	void controlCycle(float dt);

	void setRoute(struct vectors v);
	void start(void);
	void stop(void);

	void tone(int frequency, float duration);

	void printDebug(void);
};

} /* namespace RobotDev */

#endif /* ROBOT_H_ */
