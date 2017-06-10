#include "RobotConfig.h"
#include <math.h>

const float lsTheta = M_PI / 2.0;
const float lsMatrix[] = {cos(lsTheta), sin(lsTheta),  0,
		                  -sin(lsTheta), cos(lsTheta),   0,
						  -ROBOT_REAR_SENSOR_X,            ROBOT_REAR_SENSOR_Y,              1.0};

const float lfsTheta = M_PI / 6.0;
const float lfsMatrix[] = {cos(lfsTheta), sin(lfsTheta), 0,
		                  -sin(lfsTheta), cos(lfsTheta),   0,
						  0.07,            0.05,              1.0};

const float fsTheta = 0.0;
const float fsMatrix[] = {cos(fsTheta), sin(fsTheta), 0,
		                  -sin(fsTheta), cos(fsTheta),   0.0,
						  0.07,            0,              1.0};

const float rsTheta = -M_PI / 2.0;
const float rsMatrix[] = {cos(rsTheta), sin(rsTheta),  0,
		                  -sin(rsTheta), cos(rsTheta),   0,
						  -ROBOT_REAR_SENSOR_X,            -ROBOT_REAR_SENSOR_Y,              1.0};

const float rfsTheta = -M_PI / 6.0;
const float rfsMatrix[] = {cos(rfsTheta), sin(rfsTheta), 0,
		                  -sin(rfsTheta), cos(rfsTheta),   0,
						  0.07,            -0.05,              1.0};

const MatrixR leftSensorM = MatrixR(3, 3, lsMatrix);
const MatrixR leftFrontSensorM = MatrixR(3, 3, lfsMatrix);
const MatrixR frontSensorM = MatrixR(3, 3, fsMatrix);
const MatrixR rightFrontSensorM = MatrixR(3, 3, rfsMatrix);
const MatrixR rightSensorM = MatrixR(3, 3, rsMatrix);
