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

enum MOTOR_DRIVE_MODE{
    MOTOR_SLEEP,
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_ROTATE_CW,
    MOTOR_ROTATE_CCW,
};

enum WALL_SENS{
    WALL_SENS_FL,
    WALL_SENS_L,
    WALL_SENS_R,
    WALL_SENS_FR,
};


extern float gAccel[AXIS_NUM];
extern float gGyro[AXIS_NUM];

extern float gWheelAngle[SIDE_NUM];

extern int gBatteryVoltage;

extern int gIndicatorValue;

extern enum MOTOR_DRIVE_MODE gMotorDriveMode;
extern float gMotorDuty[SIDE_NUM];

extern int gWallVoltage[WALL_SENS_NUM];


#endif
