#ifndef CONTROLLER_H 
#define CONTROLLER_H

extern int straight(const float targetDistance, const float endSpeed, const float timeout,
        const float maxSpeed, const float accel);
extern int turn(const float targetAngle, const float timeout);
extern int slalom(const int isTurnRight, const float endSpeed, const float timeout);
extern int fastSlalom(const int isTurnRight, const float endSpeed, const float timeout);
extern int straightBack(const float timeout);
extern int searchStraight(const float distance, const float endSpeed);
extern int fastStraight(const float distance, const float endSpeed);
extern int ketsuate(const float endSpeed);

#endif
