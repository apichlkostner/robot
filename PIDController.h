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

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

namespace RobotDev {

class PIDController {
protected:
	float E_k;       // integrated error
	float e_k_l;     // last error
	// PID control parameter
	float Kp;
	float Ki;
	float Kd;

public:
	PIDController();
	PIDController(float Kp, float Ki, float Kd);
	virtual ~PIDController();

	float calc(float e_k, float dt);
};

} /* namespace RobotDev */

#endif
