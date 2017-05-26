#include <Arduino.h>
#include "RedBot.h"
#include <math.h>
#include "Motor.h"
#include "Robot.h"

using namespace RobotDev;
using namespace std;

#define MS2US(t) (1000ul * (t))


RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;

RedBotBumper lBumper = RedBotBumper(3);  // initialzes bumper object on pin 3
RedBotBumper rBumper = RedBotBumper(11); // initialzes bumper object on pin 11


static int doPrint;

struct vector waypoints[] = {
		{0.5, 0.0},
		{1.0, 0.5},
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
	// wait for a button press to start
	if (digitalRead(buttonPin) == LOW) {
		encoder.clearEnc(BOTH);
		robot.start();
	}

	float dt = ((float)delta_t) * 0.000001;

	robot.sense(dt);
	robot.updateState(dt);
	robot.control(dt);
	robot.actuate(dt);

	if ((doPrint++ % 3 == 0))
		robot.printDebug();

	// stop if robot collides with obstacle
	int lBumperState = lBumper.read();
	int rBumperState = rBumper.read();

	if ((lBumperState == LOW) || (rBumperState == LOW)) {
		robot.stop();
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
