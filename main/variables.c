
#include "variables.h"

enum MODE gCurrentMode = MODE_SELECT;

float gAccel[AXIS_NUM] = {0};
float gGyro[AXIS_NUM] = {0};
int gGyroBiasResetRequest; // 0 or 1

float gWheelAngle[SIDE_NUM] = {0};
float gMeasuredSpeed = 0;
float gMeasuredAngle; // radians

float gBatteryVoltage = 0;

int gIndicatorValue = 0;

enum MOTOR_STATE gMotorState = MOTOR_OFF;
float gMotorDuty[SIDE_NUM] = {0};

float gObjVoltages[OBJ_SENS_NUM] = {0};
int gIsWall[DIREC_NUM] = {0};

float gMovingDistance = 0;

float gTargetSpeed = 0; // m/s
float gTargetOmega = 0; // rad/s

int gObsBatteryIsLow = FALSE;
int gObsTouch[SIDE_NUM] = {FALSE};
float gObsMovingDistance = 0;
float gObsSpeed = 0;
float gObsWheelSpeed[SIDE_NUM] = {0};
float gObsAngle = 0;
int gObsDial = 0;
int gObsIsWall[DIREC_NUM] = {FALSE};
float gObsWallThresholds[DIREC_NUM] = {0};
float gObsWallError[SIDE_NUM] = {0};

float gLogData[LOG_DATA_NUM][LOG_INDEX_NUM] = {{0},{0},{0}};
int gLogTime[LOG_INDEX_NUM] = {0};

float gDebugValue = 0;

