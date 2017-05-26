#include <Arduino.h>
#include "RedBot.h"
#include <math.h>

#define MS2US(t) (1000ul * (t))

#define MAX_PWM 255

RedBotMotors motors;

RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;
int countsPerRev = 192;

// variables used to store the left and right encoder counts.
int lCount;
int rCount;

RedBotBumper lBumper = RedBotBumper(3);  // initialzes bumper object on pin 3
RedBotBumper rBumper = RedBotBumper(11); // initialzes bumper object on pin 11

float pos[2];
float theta;

const float R = 0.032;
const float L = 0.165;
static int doPrint;
#define NUMPOINTS 5

static float goal[NUMPOINTS][2] = {
		{1, 0},{1.5,0.6},{2.0, 0}, {1.5, -0.6}, {0,0}
};
static float E_k, e_k_l;
const float Kp = 0.08;
const float Ki = 0.01;
const float Kd = 0.02;
const float l_tire = 2.0 * M_PI * R;
static float v_s;
static int goal_nr = 0;

void setup()
{
	pinMode(buttonPin, INPUT_PULLUP);
	Serial.begin(9600);
	Serial.println("starting serial console...");
}

void loop(unsigned long delta_t)
{
	// wait for a button press to start
	if (digitalRead(buttonPin) == LOW) {
		encoder.clearEnc(BOTH);
		v_s = 0.02;
	}

	// store the encoder counts to a variable.
	int lCount_last = lCount;
	int rCount_last = rCount;
	lCount = encoder.getTicks(LEFT);    // read the left motor encoder
	rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

	int delta_l = lCount - lCount_last;
	int delta_r = rCount - rCount_last;

	float dt = ((float)delta_t) * 0.000001;
	float v_l = delta_l / dt / countsPerRev * (2.0 * M_PI);
	float v_r = delta_r / dt / countsPerRev * (2.0 * M_PI);
	float v_x = R / 2.0 * (v_r + v_l) * cos(theta);
	float v_y = R / 2.0 * (v_r + v_l) * sin(theta);
	float omega = R / L * (v_r - v_l);

	theta += omega * dt;
	theta = atan2(sin(theta), cos(theta));

	pos[0] += v_x * dt;
	pos[1] += v_y * dt;

	// vector from robot to goal
	float u_gtg[2] = {
			goal[goal_nr][0] - pos[0],
			goal[goal_nr][1] - pos[1]
	};

	// delta angle between robot heading and vector to goal
	float theta_gtg = atan2(u_gtg[1], u_gtg[0]);

	// angle error
	float e_k = theta_gtg - theta;
	e_k = atan2(sin(e_k), cos(e_k));

	// errors for PID controller
	float e_P = e_k;
	float e_I = E_k + e_k * dt;
	float e_D = (e_k - e_k_l) / dt;

	// angle speed from PID controller
	float w = Kp * e_P + Ki * e_I + Kd * e_D;


	E_k = e_I;
	// integrator should only run when robot is started
	if (v_s > 0) {
		e_k_l = e_k;
	}

	// speed for right and left wheel to set angle speed from controller
	float v_r_s = (2*v_s + w*L) / (2*R);
	float v_l_s = (2*v_s - w*L) / (2*R);

	// is the robot at the current goal?
	if (u_gtg[0]*u_gtg[0] + u_gtg[1]*u_gtg[1] < 0.01) {
		// next goal or stop when it was the last
		if (++goal_nr >= NUMPOINTS)
			v_s = 0;
	}

	// calculate pwm from velocity
	int pwm_l = 0;
	int pwm_r = 0;

	if (v_s == 0) {  // stop if skalar speed should be 0
		motors.brake();
	} else {
		int pwm_min, pwm_max;

		pwm_l = v_l_s * MAX_PWM;
		pwm_r = v_r_s * MAX_PWM;

		// handle if pwms are too large
		if (pwm_l < pwm_r) {
			pwm_min = pwm_l;
			pwm_max = pwm_r;
		} else {
			pwm_min = pwm_r;
			pwm_max = pwm_l;
		}

		if (pwm_max > MAX_PWM) {
			pwm_r -= pwm_max - 255;
			pwm_l -= pwm_max - 255;
		}
		if (pwm_min < -MAX_PWM) {
			pwm_r -= pwm_min + 255;
			pwm_l -= pwm_min + 255;
		}

		motors.leftDrive(pwm_l);
		motors.rightDrive(pwm_r);
	}


	// debug information on serial console
	if (((doPrint++) % 20) == 0) {
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
		Serial.println(delta_l);

		//		Serial.print(u_gtg[0]);
		//		Serial.print("\t");
		//		Serial.print(u_gtg[1]);
		//		Serial.print("\t");
		//		Serial.println(w);
		//
		//		Serial.print(v_l);
		//		Serial.print("\t");
		//		Serial.print(v_r);
		//		Serial.print("\t");
		//		Serial.println(v_s);
		//
		//		Serial.print(pwm_l);
		//		Serial.print("\t");
		//		Serial.println(pwm_r);
		Serial.println("");
	}

	// stop if robot collides with obstacle
	int lBumperState = lBumper.read();
	int rBumperState = rBumper.read();

	if ((lBumperState == LOW) || (rBumperState == LOW)) {
		v_s = 0;
		motors.brake();
	}
}

int main(void) {
	init();
	setup();
	unsigned long current_time = micros();
	unsigned long last_time = current_time;
	for (;;) {
		current_time = micros();
		unsigned long delta_t = (current_time - last_time);

		if (delta_t > MS2US(100ul)) {
			last_time = current_time;
			loop(delta_t);
			//Serial.println(micros() - current_time);
		} else {
		}
	}
}
