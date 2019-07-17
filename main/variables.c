
#include "variables.h"

float gAccel[AXIS_NUM] = {0};
float gGyro[AXIS_NUM] = {0};

float gWheelAngle[SIDE_NUM] = {0};
float gMeasuredSpeed = 0;

float gBatteryVoltage = 0;

int gIndicatorValue = 0;

enum MOTOR_STATE gMotorState = MOTOR_OFF;
float gMotorDuty[SIDE_NUM] = {0};

float gWallVoltage[WALL_SENS_NUM] = {0};

float gMovingDistance = 0;

enum CONTROL_REQUEST gControlRequest = CONT_NONE;
