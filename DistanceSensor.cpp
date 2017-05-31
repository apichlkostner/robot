/*
 * DistanceSensor.cpp
 *
 *  Created on: 31.05.2017
 *      Author: arthur
 */

#include "DistanceSensor.h"

namespace RobotDev {

DistanceSensor::DistanceSensor() :	pin(A0) {
}

DistanceSensor::DistanceSensor(uint8_t pin) {
	this->pin = pin;
}

DistanceSensor::~DistanceSensor() {
	// TODO Auto-generated destructor stub
}

float DistanceSensor::getDistance() {
	int sensor_adc = analogRead(pin);
	float sensor_volt = sensor_adc * 5.0 / 1024;

	int pos = 0;
	while (sensor_volt < volt[pos])
		pos++;

	float delta_dist = dist[pos] - dist[pos-1];
	float delta_volt = volt[pos-1] - volt[pos];

	float distance = dist[pos-1] + delta_dist * (volt[pos-1] - sensor_volt) / delta_volt;

	return distance;
}

} /* namespace RobotDev */
