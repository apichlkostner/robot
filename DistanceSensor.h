/*
 * DistanceSensor.h
 *
 *  Created on: 31.05.2017
 *      Author: arthur
 */

#ifndef DISTANCESENSOR_H_
#define DISTANCESENSOR_H_

#include <Arduino.h>

namespace RobotDev {

class DistanceSensor {
protected:
	uint8_t pin;

	const float volt[10]  = {3.3,  2.3, 1.3, 0.9, 0.75, 0.6, 0.5, 0.4, 0.35, 0.0};
	const float dist[10]  = {0.06, 0.1, 0.2, 0.3,  0.4, 0.5, 0.6, 0.7,  0.8, 0.8};

public:
	DistanceSensor();
	DistanceSensor(uint8_t pin);
	virtual ~DistanceSensor();

	float getDistance();
};

} /* namespace RobotDev */

#endif /* DISTANCESENSOR_H_ */
