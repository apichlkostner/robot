//    Robot control
//    Copyright (C) 2017 Arthur Pichlkostner <apichlkostner@gmx.de>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include "RedBot.h"
#include "RobotConfig.h"
#include "Motor.h"
#include "RobotSound.h"
#include "PIDController.h"
#include "DistanceSensor.h"

#ifndef ROBOTDEV_BUMPER_INSTALLED
#error "ROBOTDEV_BUMPER_INSTALLED" not defined. Please configure in RobotConfig.h
#endif
#ifndef ROBOTDEV_ACCELEROMETER_INSTALLED
#error "ROBOTDEV_ACCELEROMETER_INSTALLED" not defined. Please configure in RobotConfig.h
#endif


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

struct obstacle {
	float dist;
	MatrixR pos;
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
	float v;

	MatrixR pos;
	float theta;

	PIDController cntrl;// = PIDController(0.2, 0.05, 0.05);
	RobotSound sound;
	Motor motor;

	static const int numDistanceSensors = 5;
	DistanceSensor d_sensors[numDistanceSensors] = {DistanceSensor(A3, &leftSensorM),
			DistanceSensor(A1, &leftFrontSensorM),
			DistanceSensor(A0, &frontSensorM),
			DistanceSensor(A6, &rightFrontSensorM),
			DistanceSensor(A7, &rightSensorM)};

	struct obstacle obstacle[numDistanceSensors];
	float dist_near_obst;

	RedBotEncoder encoder = RedBotEncoder(A2, 10);

#if ROBOTDEV_BUMPER_INSTALLED
	RedBotBumper lBumper = RedBotBumper(3);
	RedBotBumper rBumper = RedBotBumper(11);
#endif
#if ROBOTDEV_ACCELEROMETER_INSTALLED
	RedBotAccel accel;
#endif

	float v_s;
	float v_r_s;
	float v_l_s;
	float w;
	float v_x;
	float v_y;
	float omega;
	float dist_goal;

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
