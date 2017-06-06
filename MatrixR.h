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

#ifndef MATRIXR_H_
#define MATRIXR_H_

namespace RobotDevMath {

class MatrixR {
public:

	MatrixR() = default;
	MatrixR(int, int, const float *);
	//MatrixR(int, int, const float *);
	MatrixR(int, int);
	bool operator==(const MatrixR& m);
	MatrixR& operator=(MatrixR&&) = default;
	MatrixR(const MatrixR&);
	MatrixR& operator=(const MatrixR&) = default;
	~MatrixR();

	MatrixR& operator+=(const MatrixR& m);
	const MatrixR operator+(const MatrixR& m);
	MatrixR& operator-=(const MatrixR& m);
	const MatrixR operator-(const MatrixR& m);
	const MatrixR operator*(const MatrixR& m) const;

	const float operator()(int col, int row) const;

private:
	int num_rows;
	int num_cols;
	float *elems;
	static constexpr float EQUAL_MAX = 0.0001;
};

} /* namespace RobotDevMath */

#endif /* MATRIXR_H_ */