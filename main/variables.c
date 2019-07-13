
#include "variables.h"

float gAccel[AXIS_NUM] = {0};
float gGyro[AXIS_NUM] = {0};

float gWheelAngle[SIDE_NUM] = {0};

int gBatteryVoltage = 0;

int gIndicatorValue = 0;

enum MOTOR_DRIVE_MODE gMotorDriveMode = MOTOR_SLEEP;
float gMotorDuty[SIDE_NUM] = {0};

int gWallVoltage[WALL_SENS_NUM] = {0};

float gMovingDistance = 0; // mm
