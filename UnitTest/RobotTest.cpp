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

#include "RobotTest.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "../Curve.h"

namespace RobotDevUnitTest {

TEST_GROUP(Curve)
						{
						};

TEST(Curve, FirstTest)
{
	float x[3] = {0, 1, 2};
	float y[3] = {1, 5, 7};
	RobotDev::Curve curve = RobotDev::Curve(x, y, 3);

	// inside curve
	DOUBLES_EQUAL(1.0, curve.getVal(0), 0.00001);
	DOUBLES_EQUAL(5.0, curve.getVal(1), 0.00001);
	DOUBLES_EQUAL(7.0, curve.getVal(2), 0.00001);
	DOUBLES_EQUAL(3.0, curve.getVal(0.5), 0.00001);
	DOUBLES_EQUAL(6.0, curve.getVal(1.5), 0.00001);
	DOUBLES_EQUAL(3.8, curve.getVal(0.7), 0.00001);
	DOUBLES_EQUAL(5.6, curve.getVal(1.3), 0.00001);
	// out of curve
	DOUBLES_EQUAL(1.0, curve.getVal(-1), 0.00001);
	DOUBLES_EQUAL(7.0, curve.getVal(8), 0.00001);
}





RobotTest::RobotTest() {

}

RobotTest::~RobotTest() {
}


} /* namespace RobotDevUnitTest */

int main(int ac, char** av)
{
	return CommandLineTestRunner::RunAllTests(ac, av);
}
