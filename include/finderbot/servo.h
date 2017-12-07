#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softPwm.h>
#define pwmpin  0
#define pwrpin  3

int servo_setup();

int servo_turn0();

int servo_turn90();

int servo_turn180();
