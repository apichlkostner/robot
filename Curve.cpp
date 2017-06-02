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

#include "Curve.h"

namespace RobotDev {

Curve::Curve() : x(0), y(0), size(0) {

}

Curve::Curve(const float *x, const float *y, const int size) : x(x), y(y), size(size) {

}

Curve::~Curve() {
}

float Curve::getVal(float v)
{
	int i = 0;
	while ((i < size) && (v > x[i]))
		i++;
	if (i == 0)
		return y[0];

	if (i == size)
		return y[size-1];

	float dx = x[i] - x[i-1];
	float dy = y[i] - y[i-1];

	return y[i-1] + (v - x[i-1]) * dy / dx;
}

} /* namespace RobotDev */
