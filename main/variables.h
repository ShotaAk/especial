#ifndef VARIABLES_H
#define VARIABLES_H

#define SIDE_NUM 2
#define AXIS_NUM 3
#define WALL_SENS_NUM 4

enum SIDE{ RIGHT=0, LEFT};
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


extern float gAccel[AXIS_NUM]; // g (9.806 m/s^2)
extern float gGyro[AXIS_NUM]; // radians/sec

extern float gWheelAngle[SIDE_NUM]; // radians

extern float gBatteryVoltage; // volts

extern int gIndicatorValue; // 0 ~ 3

extern enum MOTOR_STATE gMotorState;
extern float gMotorDuty[SIDE_NUM]; // -100 ~ +100 %

extern int gWallVoltage[WALL_SENS_NUM];

extern float gMovingDistance; // mm


#endif
