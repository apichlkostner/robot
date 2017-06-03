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

#ifndef ROBOTSOUND_H_
#define ROBOTSOUND_H_

#include <Arduino.h>
#include "RedBot.h"

namespace RobotDev {

class RobotSound {
protected:
	const int buzzerPin = 9;
	float frequency;
	float duration;

public:
	RobotSound();
	virtual ~RobotSound();

	void start(float frequency, float duration);
	void process(float dt);
};

} /* namespace RobotDev */

#endif
