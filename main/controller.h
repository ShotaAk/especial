#ifndef CONTROLLER_H 
#define CONTROLLER_H

extern void TaskControlMotion(void *arg);
extern int straight(const float targetDistance, const float endSpeed, const float timeout);

#endif
