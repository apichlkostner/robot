/*
 * Matrix.cpp
 *
 *  Created on: 03.06.2017
 *      Author: arthur
 */

#include "MatrixR.h"

#include <stdlib.h>
#include "math.h"

namespace RobotDevMath {

MatrixR::MatrixR(int num_rows, int num_cols, float *in) : num_rows(num_rows), num_cols(num_cols)
{
	int size = num_rows * num_cols;

	elems = new float[size];
	for (auto p = 0; p < size; p++) {
		elems[p] = in[p];
	}
}

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

const float MatrixR::operator()(int col, int row) const
{
	return elems[col + num_rows * row];
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

const MatrixR MatrixR::operator*(const MatrixR& m)
{
	if ((num_rows == m.num_cols)) {
		int new_x = m.num_rows;
		int new_y = num_cols;

		MatrixR mr(new_x, new_y);

		for (auto x = 0; x < new_x; x++) {
			for (auto y = 0; y < new_y; y++) {
				int pos = x + new_x * y;
				mr.elems[pos] = 0;
				for (auto i = 0; i < num_rows; i++) {
					mr.elems[pos] += (*this)(i, y) * m(x, i);
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
