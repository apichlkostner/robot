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

#include "Robot.h"
#include <math.h>

namespace RobotDev {

Robot::Robot() {
	float init_pos[] = {0, 0, 1};
	pos = MatrixR(3, 1, init_pos);
	v_s = ROBOT_V_MAX;
	started = false;
#if ROBOTDEV_ACCELEROMETER_INSTALLED
	accel.enableBump();
	accel.setBumpThresh(bumpThreshold);
#endif
}

Robot::~Robot() {
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


	float fsMatrix[] = {cos(theta), sin(theta), 0,
			-sin(theta), cos(theta),   0.0,
			pos(0, 0),            pos(1, 0),              1.0};
	MatrixR M_R2W(3, 3, fsMatrix);

	dist_near_obst = 1.0;
	for (int s = 0; s < numDistanceSensors; s++) {
		obstacle[s].dist = d_sensors[s].measure();

		if (obstacle[s].dist < dist_near_obst)
			dist_near_obst = obstacle[s].dist;

		MatrixR pos = d_sensors[s].getPosInRobotCoord();
		obstacle[s].pos  = M_R2W * pos;
	}
}


void Robot::updateState(float dt)
{
	v_x = R / 2.0 * (v_r + v_l) * cos(theta);
	v_y = R / 2.0 * (v_r + v_l) * sin(theta);
	v = sqrt(v_x*v_x + v_y*v_y);
	omega = R / L * (v_r - v_l);

	theta += omega * dt;
	theta = atan2(sin(theta), cos(theta));

	pos(0, 0) += v_x * dt;
	pos(1, 0) += v_y * dt;
}

#define CONTROLTYPE_GOTOGOAL         0
#define CONTROLTYPE_AVOIDOBSTACLE    1
#define CONTROLTYPE_BLENDING         2

#define CONTROLTYPE                  CONTROLTYPE_AVOIDOBSTACLE

void Robot::control(float dt)
{
	if (started) {
#if CONTROLTYPE == CONTROLTYPE_GOTOGOAL
		// vector from robot to goal
		vector u_gtg = {
				route.v[route.i].x - pos(0, 0),
				route.v[route.i].y - pos(1, 0)
		};
#elif CONTROLTYPE == CONTROLTYPE_AVOIDOBSTACLE
		MatrixR gtg(3, 1);
		float mposa[] = {pos(0, 0), pos(1, 0), 0};
		MatrixR mpos(3, 1, mposa);
		float weight[] = {2, 4, 3, 4, 2};

		for (int s = 0; s < numDistanceSensors; s++) {

			gtg += (obstacle[s].pos - mpos) * weight[s];
			//			Serial.print(obstacle[s].pos(0,0));
			//			Serial.print("\t");
			//			Serial.print(obstacle[s].pos(1,0));
			//			Serial.print("\t");

		}
		//		Serial.println("\t");
		//
		//		Serial.print(u_gtg.x);
		//		Serial.print("\t");
		//		Serial.print(u_gtg.y);
		//		Serial.print("\t");
		//		Serial.print(pos(0, 0));
		//		Serial.print("\t");
		//		Serial.print(pos(1, 0));
		//		Serial.print("\t");
		//		Serial.print(gtg(0,0));
		//		Serial.print("\t");
		//		Serial.print(gtg(1,0));
		//		Serial.println("");
		//		Serial.println("---------------------------");
#else
		MatrixR gtg(3, 1);
		float mposa[] = {pos(0, 0), pos(1, 0), 0};
		MatrixR mpos(3, 1, mposa);
		float weight[] = {2, 4, 3, 4, 2};

		for (int s = 0; s < numDistanceSensors; s++) {
			MatrixR obs_v = obstacle[s].pos - mpos;
			gtg += obs_v * weight[s];

		}

		float dist_front_obst = (obstacle[1].dist + obstacle[2].dist + obstacle[3].dist) / 3 / 0.8;
		float dist_obstacles = sqrt(gtg(0,0)*gtg(0,0) + gtg(1,0)*gtg(1,0));
		gtg = gtg * (1 / dist_obstacles) * (1 - dist_front_obst);

		float gtg_a[] = {
				route.v[route.i].x - pos(0, 0),
				route.v[route.i].y - pos(1, 0)
		};

		MatrixR gtg_v = MatrixR(3, 1, gtg_a);
		dist_goal = sqrt(gtg_v(0,0)*gtg_v(0,0) + gtg_v(1,0)*gtg_v(1,0));

		gtg_v = gtg_v * (1.0 / dist_goal);

		Serial.print(dist_goal);
		Serial.print("\t");
		Serial.print(dist_front_obst);
		Serial.print("\t");

		Serial.print(gtg(0,0));
		Serial.print("\t");
		Serial.print(gtg(1,0));
		Serial.print("\t");

		Serial.print(gtg_v(0,0));
		Serial.print("\t");
		Serial.print(gtg_v(1,0));
		Serial.println("\t");

		gtg += gtg_v * dist_front_obst;
#endif

		// delta angle between robot heading and vector to goal
		float theta_gtg = atan2(gtg(1,0), gtg(0,0));

		// angle error
		float e_k = theta_gtg - theta;

		e_k = atan2(sin(e_k), cos(e_k));

		// angle speed from PID controller
		w = cntrl.calc(e_k, dt);

		v_s = (obstacle[1].dist + obstacle[2].dist + obstacle[3].dist)/(3*0.8) * ROBOT_V_MAX;

#if CONTROLTYPE == CONTROLTYPE_GOTOGOAL
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
#elif CONTROLTYPE == CONTROLTYPE_AVOIDOBSTACLE
		v_r_s = v_s + w*L / (2*R);
		v_l_s = v_s - w*L / (2*R);
#else
		if (dist_goal > 0.1) {
			v_r_s = v_s + w*L / (2*R);
			v_l_s = v_s - w*L / (2*R);
		}
		else {
			route.i++;
			if (route.i >= route.size) {
				stop();
			}
		}
#endif
	}
}

void Robot::actuate(float dt)
{
	if (v_l_s != 0 || v_r_s != 0) {
		if ((v_l_s > 0) && (v_l_s < ROBOT_V_MIN))
			v_l_s = ROBOT_V_MIN;
		if ((v_l_s < 0) && (v_l_s > -ROBOT_V_MIN))
			v_l_s = -ROBOT_V_MIN;
		if ((v_r_s > 0) && (v_r_s < ROBOT_V_MIN))
			v_r_s = ROBOT_V_MIN;
		if ((v_r_s < 0) && (v_r_s > -ROBOT_V_MIN))
			v_r_s = -ROBOT_V_MIN;

		motor.setSpeeds(v_l_s / ROBOT_V_MAX, v_r_s / ROBOT_V_MAX);

		//		Serial.print(v_l_s);
		//		Serial.print("\t");
		//		Serial.print(v_r_s);
		//		Serial.print("\t");
		//		Serial.print(theta);
		//		Serial.print("\t");
		//		Serial.print(v_x);
		//		Serial.print("\t");
		//		Serial.print(v_y);
		//		Serial.print("\t");
		//		Serial.print(v);
		//		Serial.print("\t");
		//		Serial.print(w);
		//		Serial.print("\t");
		//		Serial.print(w*L / (2*R));
		//		Serial.println("\t");

	}
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

#if ROBOTDEV_BUMPER_INSTALLED
		// bumper --> stop
		int lBumperState = lBumper.read();
		int rBumperState = rBumper.read();

		if ((lBumperState == LOW) || (rBumperState == LOW)) {
			stop();
			sound.start(500, 1);
		}
#endif

#if ROBOTDEV_ACCELEROMETER_INSTALLED
		// accelerometer detects bump --> stop
		if (accel.checkBump()) {
			stop();
			sound.start(200, 1);
		}
#endif

		//		// distance sensor
		//		if (obstacle[2].dist < 0.005) {
		//			sound.start(1000, 1);
		//			stop();
		//		}
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
#if 1
	//		Serial.print(pos(0, 0));
	//		Serial.print("\t");
	//		Serial.print(pos(1, 0));
	//		Serial.print("\t");
	//		Serial.print(theta);
	//		Serial.println("\t");
	//
	//		for (int i=0; i<5; i++) {
	//			Serial.print(obstacle[i].pos(0,0));
	//			Serial.print("\t");
	//			Serial.print(obstacle[i].pos(1,0));
	//			Serial.print("\t");
	//			Serial.print(obstacle[i].pos(2,0));
	//			Serial.print("\t");
	//			Serial.print(obstacle[i].dist);
	//			Serial.println("");
	//		}
	//		Serial.println("---------------");
#else
	Serial.print(pos(0, 0));
	Serial.print("\t");
	Serial.print(pos(1, 0));
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
#endif
}

} /* namespace RobotDev */
