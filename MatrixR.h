/*
 * Matrix.h
 *
 *  Created on: 03.06.2017
 *      Author: arthur
 */

#ifndef MATRIXR_H_
#define MATRIXR_H_

namespace RobotDevMath {

class MatrixR {
public:

	MatrixR() = default;
	//MatrixR(MatrixR &&);
	MatrixR(int, int, float *);
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
	const MatrixR operator*(const MatrixR& m);

	const float operator()(int col, int row) const;

private:
	int num_rows;
	int num_cols;
	float *elems;
	static constexpr float EQUAL_MAX = 0.0001;
};

} /* namespace RobotDevMath */

#endif /* MATRIXR_H_ */
