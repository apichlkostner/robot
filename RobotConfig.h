
#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include "MatrixR.h"

using namespace RobotDevMath;

#define ROBOTDEV_ACCELEROMETER_INSTALLED    0
#define ROBOTDEV_BUMPER_INSTALLED           0

const int ROBOTDEV_NUM_DISTANCE_SENSORS = 5;
extern const MatrixR leftSensorM;
extern const MatrixR leftFrontSensorM;
extern const MatrixR frontSensorM;
extern const MatrixR rightFrontSensorM;
extern const MatrixR rightSensorM;

const float ROBOT_REAR_SENSOR_X = 0.075;
const float ROBOT_REAR_SENSOR_Y = 0.08;

#endif
