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

#include "PIDController.h"

namespace RobotDev {


PIDController::PIDController(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd)
{
}

PIDController::PIDController() : Kp(0.08), Ki(0.01), Kd(0.02)
{
}

PIDController::~PIDController()
{
}

float PIDController::calc(float e_k, float dt)
{
	float e_P = e_k;
	float e_I = E_k + e_k * dt;
	float e_D = (e_k - e_k_l) / dt;
	float w = Kp * e_P + Ki * e_I + Kd * e_D;

	e_k_l = e_k;
	E_k = e_I;

	return w;
}

} /* namespace RobotDev */
