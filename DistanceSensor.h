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

#ifndef DISTANCESENSOR_H_
#define DISTANCESENSOR_H_

#include <Arduino.h>
#include "Curve.h"
#include "MatrixR.h"

using namespace RobotDevMath;

namespace RobotDev {

class DistanceSensor {
protected:
	uint8_t pin;

	const float volt[11] = {0.35, 0.4, 0.5, 0.6, 0.75, 0.9, 1.3, 2.3, 3.3};
	static const int LEN_CURVE = sizeof(volt) / sizeof(volt[0]);
	const float dist[LEN_CURVE] = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.05};
	const MatrixR *SensorToRobot;

	Curve curve;

public:
	DistanceSensor();
	DistanceSensor(uint8_t pin);
	DistanceSensor(uint8_t pin, const MatrixR*);

	virtual ~DistanceSensor();

	float getDistance();
	MatrixR getPosInRobotCoord();
};

} /* namespace RobotDev */

#endif /* DISTANCESENSOR_H_ */
