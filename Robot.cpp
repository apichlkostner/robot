/*
 * robot.cpp
 *
 *  Created on: 26.05.2017
 *      Author: arthur
 */

#include "Robot.h"
#include <math.h>

//using namespace std;

namespace RobotDev {

Robot::Robot() {
	goal[0] = 0.5;
	goal[1] = 0.5;
	v_s = 0.02;
	started = false;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}


void Robot::sense(float dt)
{
	// store the encoder counts to a variable.
	int lCount_last = lCount;
	int rCount_last = rCount;
	lCount = encoder.getTicks(LEFT);    // read the left motor encoder
	rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

	int delta_l = lCount - lCount_last;
	int delta_r = rCount - rCount_last;

	v_l = delta_l / dt / countsPerRev * (2.0 * M_PI);
	v_r = delta_r / dt / countsPerRev * (2.0 * M_PI);
}


void Robot::updateState(float dt)
{
	v_x = R / 2.0 * (v_r + v_l) * cos(theta);
	v_y = R / 2.0 * (v_r + v_l) * sin(theta);
	omega = R / L * (v_r - v_l);

	theta += omega * dt;
	theta = atan2(sin(theta), cos(theta));

	pos[0] += v_x * dt;
	pos[1] += v_y * dt;
}

void Robot::control(float dt)
{
	if (started) {
		// vector from robot to goal
		float u_gtg[2] = {
				route.v[route.i].x - pos[0],
				route.v[route.i].y - pos[1]
		};

		// delta angle between robot heading and vector to goal
		float theta_gtg = atan2(u_gtg[1], u_gtg[0]);

		// angle error
		float e_k = theta_gtg - theta;
		e_k = atan2(sin(e_k), cos(e_k));

		// errors for PID controller
		float e_P = e_k;
		float e_I = cntrl.E_k + e_k * dt;
		float e_D = (e_k - cntrl.e_k_l) / dt;

		// angle speed from PID controller
		w = cntrl.Kp * e_P + cntrl.Ki * e_I + cntrl.Kd * e_D;

		cntrl.e_k_l = e_k;

		cntrl.E_k = e_I;


		// speed for right and left wheel to set angle speed from controller

		if (u_gtg[0]*u_gtg[0] + u_gtg[1]*u_gtg[1] > 0.1) {
			v_r_s = (2*v_s + w*L) / (2*R);
			v_l_s = (2*v_s - w*L) / (2*R);
		} else {
			route.i++;
			if (route.i >= route.size) {
				started = false;
				v_r_s = 0;
				v_l_s = 0;
			}
		}
	}
}

void Robot::actuate(float dt)
{
	if (v_l_s != 0 || v_r_s != 0)
		motor.setSpeeds(v_l_s, v_r_s);
	else
		motor.brake();
}

void Robot::setRoute(struct vectors v)
{
	route = v;
}

void Robot::start()
{
	started = true;
}

void Robot::stop()
{
	started = false;
	v_r_s = 0;
	v_l_s = 0;
}

void Robot::printDebug()
{
	Serial.print(pos[0]);
	Serial.print("\t");
	Serial.print(pos[1]);
	Serial.print("\t");
	Serial.println(theta);

	Serial.print(v_x);
	Serial.print("\t");
	Serial.print(v_y);
	Serial.print("\t");
	Serial.println(omega);

	Serial.print(v_r);
	Serial.print("\t");
	Serial.print(v_l);
	Serial.print("\t");
	Serial.println(route.i);

	Serial.print(goal[0]);
	Serial.print("\t");
	Serial.print(goal[1]);
	Serial.print("\t");
	Serial.println(w);

	Serial.print(v_l);
	Serial.print("\t");
	Serial.print(v_r);
	Serial.print("\t");
	Serial.println(v_s);

	Serial.println("");
}

} /* namespace robot */
