
#ifndef MOTOR_h
#define MOTOR_h

#include <inttypes.h>
/* speeds updated by PID and MotorCommand */
extern volatile int speedFL;
extern volatile int speedFR;
extern volatile int speedBL;
extern volatile int speedBR;


// /* Motor enable pins */
// #define EN1  PD7
// #define EN2  PB6
// #define EN3  PC7
// #define EN4  PF7

/* motor commands */
#define STOP  'S'
#define FORWARD  'F'
#define BACKWARD  'B'
#define RIGHT 'R'
#define LEFT 'L'

extern volatile char command_state;

void init_pwm();

void moveRobot(char command);

void motor_run(int motor, uint8_t speed, int movement);

void stop_bot(void);

void forwards(void);

void backwards(void);

void right(void);

void left(void);

void enable_motors(void);

void disable_motors(void);

void update_speed(void);

void init_motor_pins(void);

void reset_speeds(void);


#endif