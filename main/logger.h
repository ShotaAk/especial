#ifndef LOGGER_H
#define LOGGER_H

#include "variables.h"

extern void loggingInitialize(const int period_msec, const int timeout_msec,
        char *name1, float *data1,
        char *name2, float *data2,
        char *name3, float *data3);
extern void loggingStart(void);
extern void loggingStop(void);
extern void loggingReset(void);
extern int loggingIsInitialized(void);
extern int loggingIsStarted(void);
extern void loggingPrint(void);
extern void loggingSave(void);
extern void loggingLoadPrint(void);
extern void loggingSaveWall(const int mazesize_x, const int mazesize_y, t_wall wallMap[mazesize_x][mazesize_y]);
extern void loggingLoadPrintWall(void);
extern void TaskLogging(void *arg);

#endif 
