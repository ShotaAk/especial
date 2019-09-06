#ifndef LOGGER_H
#define LOGGER_H

extern void loggingInitialize(const int period_msec, const int timeout_msec,
        char *name1, float *data1);
extern void loggingStart(void);
extern void loggingStop(void);
extern void loggingReset(void);
extern int loggingIsInitialized(void);
extern int loggingIsStarted(void);
extern void loggingPrint(void);
extern void loggingSave(void);
extern void loggingLoadPrint(void);
extern void TaskLogging(void *arg);

#endif 
