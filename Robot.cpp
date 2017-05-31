/*
 * robot.cpp
 *
 *  Created on: 26.05.2017
 *      Author: Arthur Pichlkostner
 */

#include "Robot.h"
#include <math.h>

namespace RobotDev {

Robot::Robot() {
	v_s = 0.02;
	started = false;
	accel.enableBump();
	accel.setBumpThresh(bumpThreshold);
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

	pos.x += v_x * dt;
	pos.y += v_y * dt;
}

void Robot::control(float dt)
{
	if (started) {
		// vector from robot to goal
		vector u_gtg = {
				route.v[route.i].x - pos.x,
				route.v[route.i].y - pos.y
		};

		// delta angle between robot heading and vector to goal
		float theta_gtg = atan2(u_gtg.y, u_gtg.x);

		// angle error
		float e_k = theta_gtg - theta;

		e_k = atan2(sin(e_k), cos(e_k));

		// angle speed from PID controller
		w = cntrl.calc(e_k, dt);

		// speed for right and left wheel to set angle speed from controller
		if (u_gtg.x*u_gtg.x + u_gtg.y*u_gtg.y > stopDistanceSquared) {
			v_r_s = (2*v_s + w*L) / (2*R);
			v_l_s = (2*v_s - w*L) / (2*R);
		} else {
			route.i++;
			if (route.i >= route.size) {
				stop();
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

void Robot::checkState(float dt)
{
	// stops the robot in case of crash
	if (started) {
		// wheels are stuck --> stop
		static float stuck_time = 0.0;

		if (v_l == 0 && v_r == 0) {
			stuck_time += dt;

			if (stuck_time > 0.5) {
				stop();
				sound.start(2000, 1);
			}
		} else {
			stuck_time = 0.0;
		}

		// bumper --> stop
		int lBumperState = lBumper.read();
		int rBumperState = rBumper.read();

		if ((lBumperState == LOW) || (rBumperState == LOW)) {
			stop();
			sound.start(500, 1);
		}

		// accelerometer detects bump --> stop
		if (accel.checkBump()) {
			stop();
			sound.start(200, 1);
		}

		float front_distance = d_sensors[0].getDistance();

		if (front_distance < 0.1) {
			sound.start(1000, 1);
			stop();
		}
	}
}

void Robot::controlCycle(float dt)
{
	sense(dt);
	updateState(dt);
	checkState(dt);
	control(dt);
	actuate(dt);
	sound.process(dt);

	//	Serial.print(d_sensors[0].getDistance());
	//	Serial.print("\t");
	//	Serial.println(d_sensors[1].getDistance());
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
	Serial.print(pos.x);
	Serial.print("\t");
	Serial.print(pos.y);
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

	Serial.print(route.v[route.i].x);
	Serial.print("\t");
	Serial.print(route.v[route.i].y);
	Serial.print("\t");
	Serial.println(w);

	Serial.print(v_l);
	Serial.print("\t");
	Serial.print(v_r);
	Serial.print("\t");
	Serial.println(v_s);

	Serial.println("");
}

} /* namespace RobotDev */
