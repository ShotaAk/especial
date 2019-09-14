#ifndef CONTROLLER_H 
#define CONTROLLER_H

extern int straight(const float targetDistance, const float endSpeed, const float timeout,
        const float maxSpeed, const float accel);
extern int turn(const float targetAngle, const float timeout);
extern void TaskControlMotion(void *arg);

#endif
