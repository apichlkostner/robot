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

#include "RobotSound.h"

namespace RobotDev {

RobotSound::RobotSound() {

}

RobotSound::~RobotSound() {
}

void RobotSound::start(float frequency, float duration)
{
	this->frequency = frequency;
	this->duration = duration;

	tone(buzzerPin, frequency);
}

void RobotSound::process(float dt)
{
	if (duration > 0.0) {
		duration -= dt;
	} else {
		noTone(buzzerPin);
	}
}

} /* namespace RobotDev */
