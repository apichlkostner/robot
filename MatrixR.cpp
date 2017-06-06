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

#include "MatrixR.h"

#include <stdlib.h>
#include "math.h"

namespace RobotDevMath {

MatrixR::MatrixR(int num_rows, int num_cols, const float *in) : num_rows(num_rows), num_cols(num_cols)
{
	int size = num_rows * num_cols;

	elems = new float[size];
	for (auto p = 0; p < size; p++) {
		elems[p] = in[p];
	}
}

//MatrixR::MatrixR(int num_rows, int num_cols, const float *in) : num_rows(num_rows), num_cols(num_cols), elems(in)
//{
//	int size = num_rows * num_cols;
//}


MatrixR::MatrixR(int num_rows, int num_cols) : num_rows(num_rows), num_cols(num_cols)
{
	int size = num_rows * num_cols;

	elems = new float[size];
}


MatrixR::MatrixR(const MatrixR& m) : num_rows(m.num_rows), num_cols(m.num_cols)
{
	int size = num_rows * num_cols;
	elems = new float[size];
	for (auto p = 0; p < size; p++) {
		elems[p] = m.elems[p];
	}
}

MatrixR::~MatrixR()
{
	if (elems)
		delete[] elems;
}

const float MatrixR::operator()(int row, int col) const
{
	return elems[row + num_rows * col];
}

bool MatrixR::operator==(const MatrixR& m)
								{
	if (num_rows != m.num_rows || num_cols != m.num_cols)
		return false;

	int size = num_rows * num_cols;
	for (auto p = 0; p < size; p++) {
		if (abs(elems[p] - m.elems[p]) > EQUAL_MAX)
			return false;

	}
	return true;
								}

MatrixR& MatrixR::operator+=(const MatrixR& m)
								{
	if ((m.num_rows == num_rows) && (m.num_cols == num_cols)) {
		int size = num_rows * num_cols;
		for (auto p = 0; p < size; p++) {
			elems[p] += m.elems[p];
		}
	}
	return *this;
								}

MatrixR& MatrixR::operator-=(const MatrixR& m)
								{
	if ((m.num_rows == num_rows) && (m.num_cols == num_cols)) {
		int size = num_rows * num_cols;
		for (auto p = 0; p < size; p++) {
			elems[p] -= m.elems[p];
		}
	}
	return *this;
								}

const MatrixR MatrixR::operator*(const MatrixR& m) const
{
	if ((num_cols == m.num_rows)) {
		int new_rows = num_rows;
		int new_cols = m.num_cols;

		MatrixR mr(new_rows, new_cols);

		for (auto r = 0; r < new_rows; r++) {
			for (auto c = 0; c < new_cols; c++) {
				int pos = r + new_rows * c;
				mr.elems[pos] = 0;
				for (auto i = 0; i < num_cols; i++) {
					mr.elems[pos] += (*this)(r, i) * m(i, c);
				}
			}
		}

		return mr;
	}
	return MatrixR();
}


const MatrixR MatrixR::operator+(const MatrixR& m)
{
	return MatrixR(*this) += m;
}

const MatrixR MatrixR::operator-(const MatrixR& m)
{
	return MatrixR(*this) -= m;
}


} /* namespace RobotDevMath */
