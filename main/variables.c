
#include "variables.h"

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

enum CONTROL_REQUEST gControlRequest = CONT_FINISH;

int gObsBatteryIsLow = FALSE;
int gObsTouch[SIDE_NUM] = {FALSE};
float gObsMovingDistance = 0;
float gObsSpeed = 0;
float gObsAngle = 0;
int gObsDial = 0;

float gLogData[LOG_NUM] = {0};
int gLogTime[LOG_NUM] = {0};

