#ifndef PID_h
#define PID_h

#include "PID_v1.h"
#include "imu.h"
#include "motor.h"

//Location PID CONTROL 
extern float Kp;
extern float Ki;
extern float Kd;

extern double Setpoint, Output, Input;

extern float_bytes desired_heading_num;

void init_pid(void);
void limit_speeds(void);

extern PID myPID;

#endif