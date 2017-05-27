#include <Arduino.h>
#include "RedBot.h"
#include <math.h>
#include "Motor.h"
#include "Robot.h"

using namespace RobotDev;
using namespace std;

#define MS2US(t) (1000ul * (t))

int buttonPin = 12;

static int doPrint;

struct vector waypoints[] = {
		{1.0, 0.0},
		{0.0, 0.0},
};

// route the robot should follow
struct vectors route;

Robot robot;

void setup()
{
	pinMode(buttonPin, INPUT_PULLUP);
	Serial.begin(9600);
	Serial.println("Robot: starting serial console...");

	route.v = waypoints;
	route.i = 0;
	route.size = 2;
	robot.setRoute(route);
}

void loop(unsigned long delta_t)
{
	float dt = ((float)delta_t) * 0.000001;

	// wait for a button press to start
	if (digitalRead(buttonPin) == LOW) {
		robot.start();
	}

	robot.controlCycle(dt);

	if ((doPrint++ % 3 == 0))
		robot.printDebug();
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
