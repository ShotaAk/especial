#ifndef VARIABLES_H
#define VARIABLES_H

#define TRUE 1
#define FALSE 0

#define SIDE_NUM 2
#define DIREC_NUM 4
#define AXIS_NUM 3
#define OBJ_SENS_NUM 4
#define LOG_NUM 4096

enum SIDE{ RIGHT=0, LEFT};
enum DIRECTION{
    DIREC_FRONT,
    DIREC_LEFT,
    DIREC_RIGHT,
    DIREC_BACK,
};

enum AXIS{
    AXIS_X=0,
    AXIS_Y,
    AXIS_Z
};

enum MOTOR_STATE{
    MOTOR_ON,
    MOTOR_OFF
};

enum OBJ_SENS{
    OBJ_SENS_FL,
    OBJ_SENS_L,
    OBJ_SENS_R,
    OBJ_SENS_FR,
};

enum CONTROL_REQUEST{
    CONT_NONE,
    CONT_FORWARD,
    CONT_HALF_FORWARD,
    CONT_TURN_LEFT,
    CONT_TURN_RIGHT,
    CONT_TURN_BACK,
    CONT_STOP,
    CONT_KETSUATE,
    CONT_ENKAI,
    CONT_FINISH,
};


extern float gAccel[AXIS_NUM]; // g (9.806 m/s^2)
extern float gGyro[AXIS_NUM]; // radians/sec
extern int gGyroBiasResetRequest; // 0 or 1

extern float gWheelAngle[SIDE_NUM]; // radians
extern float gMeasuredSpeed; // m/s
extern float gMeasuredAngle; // radians

extern float gBatteryVoltage; // volts

extern int gIndicatorValue; // 0 ~ 3

extern enum MOTOR_STATE gMotorState;
extern float gMotorDuty[SIDE_NUM]; // -100 ~ +100 %

extern float gObjVoltages[OBJ_SENS_NUM]; // volts
extern int gIsWall[DIREC_NUM];

extern float gMovingDistance; // meters

extern enum CONTROL_REQUEST gControlRequest;

extern int gObsBatteryIsLow; // 0 or 1
extern int gObsTouch[SIDE_NUM]; // 0 or 1

extern float gLogData[LOG_NUM];
extern int gLogTime[LOG_NUM];

#endif
