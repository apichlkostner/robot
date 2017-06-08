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

#include "DistanceSensor.h"

namespace RobotDev {

DistanceSensor::DistanceSensor() {
	DistanceSensor(A0);
}

DistanceSensor::DistanceSensor(uint8_t pin, const MatrixR *MPos) {
	curve = Curve(volt, dist, LEN_CURVE);
	this->pin = pin;
	SensorToRobot = MPos;
}

DistanceSensor::~DistanceSensor() {
	// TODO Auto-generated destructor stub
}

float DistanceSensor::measure() {
	int sensor_adc = analogRead(pin);
	float sensor_volt = sensor_adc * 5.0 / 1024;

	distance = curve.getVal(sensor_volt);

	return distance;
}


float DistanceSensor::getDistance() {
	return distance;
}

MatrixR DistanceSensor::getPosInRobotCoord()
{
	float dist_vec[] = {distance, 0, 1};
	MatrixR posInSensor = MatrixR(3, 1, dist_vec);

	return (*SensorToRobot) * posInSensor;
}

} /* namespace RobotDev */
