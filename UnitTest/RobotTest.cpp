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
#include <math.h>

#include "../RobotConfig.h"
#include "../Curve.h"
#include "../MatrixR.h"
#include "../DistanceSensor.h"

int analogRead(int r)
{
	return 500;
}

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

TEST_GROUP(MatrixR)
{
};

TEST(MatrixR, FirstTest)
{
	float init_1[] = {1, 0.5, 2, 0.3};

	float init_2[] = {-0.5, 1, 1.3, -2};
	float init_2_cpy[] = {-0.5, 1, 1.3, -2};

	float init_result[] = {0.5, 1.5, 3.3, -1.7};
	float init_result_mult[] = {1.5, 0.05, -2.7, 0.05};

	RobotDevMath::MatrixR m1(2, 2, init_1);
	RobotDevMath::MatrixR m2(2, 2, init_2);
	RobotDevMath::MatrixR m2_cpy(2, 2, init_2_cpy);
	RobotDevMath::MatrixR m_add_result(2, 2, init_result);
	RobotDevMath::MatrixR m_mult_result(2, 2, init_result_mult);

	// test access of matrix elements
	DOUBLES_EQUAL(1.0, m1(0,0), 0.000001);
	DOUBLES_EQUAL(0.5, m1(1,0), 0.000001);
	DOUBLES_EQUAL(2.0, m1(0,1), 0.000001);
	DOUBLES_EQUAL(0.3, m1(1,1), 0.000001);

	DOUBLES_EQUAL(-0.5, m2(0,0), 0.000001);
	DOUBLES_EQUAL(1.0, m2(1,0), 0.000001);
	DOUBLES_EQUAL(1.3, m2(0,1), 0.000001);
	DOUBLES_EQUAL(-2.0, m2(1,1), 0.000001);

	// check of equality operator
	CHECK(m2 == m2_cpy);
	CHECK_FALSE(m2 == m1);

	// check of += operator
	m2_cpy += m1;
	CHECK(m2_cpy == m_add_result);

	// check of -= operator
	m2_cpy -= m1;
	CHECK(m2_cpy == m2);

	// check of + operator
	RobotDevMath::MatrixR mr = m1 + m2;
	CHECK(mr == m_add_result);

	// check of * operator
	RobotDevMath::MatrixR mm = m1 * m2;
	CHECK(mm == m_mult_result);


	// positive and negative test for matrix vector multiplication
	float init_vector[] = {0.2, 0.8};
	float init_vector_result[] = {1.8, 0.34};
	float init_vector_notresult[] = {0.6, 0.63};
	RobotDevMath::MatrixR v1(2, 1, init_vector);
	RobotDevMath::MatrixR mv = m1 * v1;

	CHECK(mv == RobotDevMath::MatrixR(2, 1, init_vector_result));
	CHECK_FALSE(mv == RobotDevMath::MatrixR(2, 1, init_vector_notresult));

	// translation and rotation of 2D vector in homogeneous coodinates
	// result calculated with Matlab
	float point[] = {5.0, 2.0, 1};  // homogeneous coordinate
	float translation[2] = {0.2, 0.8};
	float theta = M_PI / 4;
	float result[] = {2.3213, 5.7497, 1.0};

	float transformation[] = {cos(theta), sin(theta), 0,
			-sin(theta), cos(theta), 0,
			translation[0], translation[1], 1};
	RobotDevMath::MatrixR trans_m(3, 3, transformation);
	RobotDevMath::MatrixR vec_p(3, 1, point);
	RobotDevMath::MatrixR vec_n = trans_m * vec_p;
	RobotDevMath::MatrixR vec_r(3, 1, result);

	CHECK(vec_n == vec_r);

}



TEST_GROUP(DistanceSensor)
{

};

TEST(DistanceSensor, FirstTest)
{
	RobotDev::DistanceSensor(0, &leftFrontSensorM);

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
