#include <avr/io.h>
#include <util/delay.h>
#include "Arduino_FreeRTOS.h"

#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "FreeSixIMU.h"


void init_pwm();

void motor_run(int motor, uint8_t speed, int movement);

void stop_bot(void);

void forwards(void);

void backwards(void);

void right(void);

void left(void);

void enable_motors(void);

void update_speed(void);

void init_motor_pins(void);

void reset_speeds(void);

void TaskMotorCommand(void *pvParameters);

void TaskIRSensorRead(void *pvParameters);

void updateAngle(void);

void TaskIMURead(void *pvParameters);

void TaskSendSPIData(void *pvParameters);

void TaskPIDController(void *pvParameters);

void TaskTestTimers(void *pvParameters);

/* speeds updated by PID and MotorCommand */
volatile int speedFL = 0;
volatile int speedFR = 0;
volatile int speedBL = 0;
volatile int speedBR = 0;

/* values read from IR sensor. Updated by IRSensorRead. Sent via SPI to XU4 */
int IRleftVal = 0;
int IRrightVal = 0;

/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
int roll = 0;
int pitch = 0;
int yaw = 0;

/* Motor pin array */
int Motor[4][2] = //two dimensional array
{
{PD4 , PD6},   //input pin to control Motor1 (front right)
{PB4 , PB5},   //input pin to control Motor2 (back right)
{PF4 , PF5},   //input pin to control Motor3 (front left)
{PF6 , PF7},  //input pin to control Motor4 (back left)
};

/* Motor enable pins */
#define EN1  PD7
#define EN2  PB6
#define EN3  PC7
#define EN4  PF7

/* motor commands */
#define STOP  'S'
#define FORWARD  'F'
#define BACKWARD  'B'
#define RIGHT 'R'
#define LEFT 'L'

/* motor command overhead */
const char SoP = 'C';
const char EoP = 'E';
const char nullTerminator = '\0';
unsigned char inByte;
#define MESSAGE_MAX_SIZE 5
char message[MESSAGE_MAX_SIZE];
char command;

/*IMU constants & objects*/ 

FreeSixIMU sixDOF = FreeSixIMU();


const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

// time vars
int currTime = 0; 
int prevTime = 0; 

float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
  float Lp = 0.5;
  float Li = 0.05;
  float Ld = 0.4;
  float offsetLoc = 0;
  float pT,iT,dT = 0;
  float errorS = 0;
  float prevE = 0;

#define F_CPU 16000000

int main(void)
{

    // xTaskCreate(
    // TaskTestTimers
    // ,  (const portCHAR *)"TimerTest"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  5  // Priority (low num = low priority)
    // ,  NULL );

    xTaskCreate(
    TaskMotorCommand
    ,  (const portCHAR *)"MotorCommand"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  5  // Priority (low num = low priority)
    ,  NULL );

    // xTaskCreate(
    // TaskIRSensorRead
    // ,  (const portCHAR *)"IRSensorRead"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  3  // Priority (low num = low priority)
    // ,  NULL );

    xTaskCreate(
    TaskIMURead
    ,  (const portCHAR *)"IMURead"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority (low num = low priority)
    ,  NULL );

    // xTaskCreate(
    // TaskSendSPIData
    // ,  (const portCHAR *)"SendSPIData"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  2  // Priority (low num = low priority)
    // ,  NULL );

    // xTaskCreate(
    // TaskPIDController
    // ,  (const portCHAR *)"PIDController"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  1  // Priority (low num = low priority)
    // ,  NULL );


    vTaskStartScheduler();

    return 0;               /* never reached */
}

void init_pwm() {
    DDRC |= _BV(PC7) | _BV(PC6); //set DDC7 = 1 -> PC7 is output pin, DDC6 = 1 ->PC6 is output
    DDRD |= _BV(PD7); //set DDD0 as output, testing; DDD7 as output -> PD7
    DDRB |= _BV(PB6); //set DDB6 as output -> PB6


    TCCR4C |= _BV(COM4D1) | _BV(PWM4D); // set timer 4 to clear when counting up, set when counting down for PD7
    
    TCCR4A |= _BV(PWM4A) | _BV(COM4A1) | _BV(PWM4B)| _BV(COM4B1); 
                //initiallize Timer 4 in PWM mode for OCR4A and OCR4B, 
               //clear when counting up, set when counting down for A & B
    TCCR4B |= _BV(CS40) | _BV(CS41) | _BV(CS42); //initialize counter 4 with divide by 64 prescaler -> 490 Hz PWM
    
    TCCR4D |= _BV(WGM40); //set Timer 4 for phase and freq correct mode

    OCR4C = 0xFF; //set TOP value for Timer 4


    TCCR3A |=  _BV(COM3A1); //initiallize Timer 3 to clear when counting up, set when coungting down

    TCCR3B |= _BV(CS30) | _BV(CS31) | _BV(WGM33); //initialize counter 3 with  divide by 64 prescaler -> 490 Hz PWM
                                                //set WGM33 to 1 to make freq/phase correct PWM

    ICR3 = 0xFF; //set TOP value for Timer 3
    
}

void motor_run(int motor, uint8_t speed, int movement) {
    if(movement == STOP) {
        speedFR = 0; speedBR = 0; speedFL = 0; speedBL = 0;
        speed = 0;
        PORTD &= ~(_BV(PD4)) & ~(_BV(PD6)); //low
        PORTB &= ~(_BV(PB4)) & ~(_BV(PB5));
        PORTF &= ~(_BV(PF4)) & ~(_BV(PF5)) & ~(_BV(PF6)) & ~(_BV(PF7));

        update_speed();
    }
    else {
        switch(motor) {
        case 0: 
            speedFR = speed;
            switch (movement) {
                case FORWARD:  
                  PORTD |= _BV(PD4); //high
                  PORTD &= ~(_BV(PD6)); //low
                  break;
                case BACKWARD:   
                  PORTD &= ~(_BV(PD4)); //low
                  PORTD |= _BV(PD6); //high
                  break; 
            }
            break;
        case 1:
            speedBR = speed;
            switch (movement) {
                case FORWARD:  
                  PORTB |= _BV(PB4); //high
                  PORTB &= ~(_BV(PB5)); //low
                  break;
                case BACKWARD:   
                  PORTB &= ~(_BV(PB4)); //low
                  PORTB |= _BV(PB5); //high
                  break; 
            }
            break;
        case 2:
            speedFL = speed;
            switch (movement) {
                case FORWARD:  
                  PORTF |= _BV(PF4); //high
                  PORTF &= ~(_BV(PF5)); //low
                  break;
                case BACKWARD:   
                  PORTF &= ~(_BV(PF4)); //low
                  PORTF |= _BV(PF5); //high
                  break; 
            }
            break;
        case 3:
            speedBL = speed;
            switch (movement) {
                case FORWARD:  
                  PORTF |= _BV(PF6); //high
                  PORTF &= ~(_BV(PF7)); //low
                  break;
                case BACKWARD:   
                  PORTF &= ~(_BV(PF6)); //low
                  PORTF |= _BV(PF7); //high
               
            break;
        }
        update_speed();
    }
  
  } 
}

void enable_motors(void) {
    TCCR4A |= _BV(COM4A1) | _BV(COM4B1);
    TCCR4C |= _BV(COM4D1);
    TCCR3A |= _BV(COM3A1);
}

void update_speed(void) {
    OCR4D = speedFR;
    OCR4B = speedBR;
    OCR4A = speedFL;
    OCR3A = speedBL;
}

void stop_bot(void) {
    TCCR4A &= ~(_BV(COM4A1)) & ~(_BV(COM4B1));
    TCCR4C &= ~(_BV(COM4D1));
    TCCR3A &=  ~(_BV(COM3A1));

    motor_run(0, speedFR, STOP);
    motor_run(1, speedBR, STOP);
    motor_run(2, speedFL, STOP);
    motor_run(3, speedBL, STOP);
}

void forwards(void) {
    enable_motors();
    motor_run(0, speedFR, FORWARD); 
    motor_run(1, speedBR, FORWARD); 
    motor_run(2, speedFL, FORWARD); 
    motor_run(3, speedBL, FORWARD); 
}

void backwards(void){
    enable_motors();
    motor_run(0, speedFR, BACKWARD);
    motor_run(1, speedBR, BACKWARD);
    motor_run(2, speedFL, BACKWARD);
    motor_run(3, speedBL, BACKWARD); 
}

void right(void) {
    enable_motors();
    motor_run(0, speedFR, BACKWARD); 
    motor_run(1, speedBR, BACKWARD);    
    motor_run(2, speedFL, FORWARD);    
    motor_run(3, speedBL, FORWARD);    
}

void left(void) {
    enable_motors();
    motor_run(0, speedFR, FORWARD);    
    motor_run(1, speedBR, FORWARD);    
    motor_run(2, speedFL, BACKWARD);    
    motor_run(3, speedBL, BACKWARD);
}

void moveRobot(char command) {
     switch(command) {
        case FORWARD:
            forwards();
            break;
        case BACKWARD:
            backwards();
            break;
        case STOP:
            stop_bot();
            break;
        case RIGHT:
            right();
            break;
        case LEFT:
            left();
            break;
        default:
            stop_bot();
            break;
    }
}

void init_motor_pins(void) {
    DDRD |= _BV(PD4) | _BV(PD6);
    DDRB |= _BV(PB4) | _BV(PB5);
    DDRF |= _BV(PF4) | _BV(PF5) | _BV(PF6) | _BV(PF7);
}

void reset_speeds(void) {
    speedFR = 0x7F; //set initial 50% duty cycle 
    speedBR = 0x7F; //set initial 50% duty cycle
    speedFL = 0x7F; //set initial 50% duty cycle 
    speedBL = 0x7F; //set initial 50% duty cycle
    update_speed();
}

void TaskMotorCommand( void *pvParameters)  // This is a Task.
{
    init_pwm();

    init_motor_pins();

    reset_speeds();

    for (;;) // A Task shall never return or exit. 
    {
        moveRobot(FORWARD);
        vTaskDelay(pdMS_TO_TICKS(500));

        moveRobot(STOP);
        vTaskDelay(pdMS_TO_TICKS(500));

        reset_speeds();

        moveRobot(BACKWARD);
        vTaskDelay(pdMS_TO_TICKS(500));

        moveRobot(STOP);
        vTaskDelay(pdMS_TO_TICKS(500));

        reset_speeds();

    }
}

void TaskIRSensorRead(void *pvParameters) {

}


void updateAngle(void) {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}

void TaskIMURead(void *pvParameters) {
    sixDOF.init(); //Begin the IMU
    for(;;) {
        updateAngle();
    }
}

void TaskSendSPIData(void *pvParameters) {

}

void TaskPIDController(void *pvParameters) {

}


// void TaskTestTimers(void *pvParameters) {
    
//     init_pwm();

//     DDRD |= _BV(PD0); //just for testing

//     speedFR = 0x7F; //set initial 50% duty cycle 
//     speedBR = 0x7F; //set initial 50% duty cycle
//     speedFL = 0x7F; //set initial 50% duty cycle 
//     speedBL = 0x7F; //set initial 50% duty cycle
//     update_speed();

//     for(;;)
//     {
//         // OCR3A = 0x7F; //set duty cycle for PC6 - TOP is 0xFF by default. 
//         // OCR4A = 0x7F; //set duty cycle for PC7 
//         // OCR4B = 0x7F; //set duty cycle for PB6
//         // OCR4D = 0x7F; //set duty cycle for PB7

//         vTaskDelay(pdMS_TO_TICKS(100));
//         PORTD ^= _BV(PD0);    /* toggle the LED */
//     }
// }