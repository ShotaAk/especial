#ifndef VARIABLES_H
#define VARIABLES_H

#define TRUE 1
#define FALSE 0

#define SIDE_NUM 2
#define DIREC_NUM 4
#define AXIS_NUM 3
#define WALL_SENS_NUM 4

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

enum WALL_SENS{
    WALL_SENS_FL,
    WALL_SENS_L,
    WALL_SENS_R,
    WALL_SENS_FR,
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

extern float gWallVoltage[WALL_SENS_NUM]; // volts
extern int gIsWall[DIREC_NUM];

extern float gMovingDistance; // meters

extern enum CONTROL_REQUEST gControlRequest;

extern int gObsBatteryIsLow; // 0 or 1

#endif
