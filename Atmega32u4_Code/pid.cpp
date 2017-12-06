#include "pid.h"

float Ki = 5;
float Kd = 1;
float Kp = 2;

double Setpoint, Output, Input;

PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd,P_ON_M, DIRECT);

float_bytes desired_heading_num;

void init_pid(void) {
	if(desired_heading_num.num_float) {
		Setpoint = (double) desired_heading_num.num_float;
	}
	else {
		if(heading.num_float)
			Setpoint = (double) heading.num_float;
		else
			Setpoint = 0;
	}

	if(heading.num_float) {
		Input = (double) heading.num_float;
	}
	else {
		Input = 0;
	}

	myPID.SetOutputLimits((-64), (64));

	//turn the PID on
  	myPID.SetMode(AUTOMATIC);

}

void limit_speeds(void) {
	speedFL = (speedFL > MAX_SPEED) ? MAX_SPEED : speedFL;
	speedFL = (speedFL < MIN_SPEED) ? MIN_SPEED : speedFL;

	speedBL = (speedBL > MAX_SPEED) ? MAX_SPEED : speedBL;
	speedBL = (speedBL < MIN_SPEED) ? MIN_SPEED : speedBL;

	speedFR = (speedFR > MAX_SPEED) ? MAX_SPEED : speedFR;
	speedFR = (speedFR < MIN_SPEED) ? MIN_SPEED : speedFR;

	speedBR = (speedBR > MAX_SPEED) ? MAX_SPEED : speedBR;
	speedBR = (speedBR < MIN_SPEED) ? MIN_SPEED : speedBR;
}