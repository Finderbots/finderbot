#include <avr/io.h>
#include <util/delay.h>
#include "Arduino_FreeRTOS.h"

void TaskMotorCommand(void *pvParameters);

void TaskIRSensorRead(void *pvParameters);

void TaskIMURead(void *pvParameters);

void TaskSendSPIData(void *pvParameters);

void TaskPIDController(void *pvParameters);

void TaskTestTimers(void *pvParameters);

/* speeds updated by PID and MotorCommand */
int speedFL = 0;
int speedFR = 0;
int speedBL = 0;
int speedBR = 0;

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
{4 , 5},   //input pin to control Motor1 (front right)--> Motor[0][0]=4, Motor[0][1]=5
{6 , 7},   //input pin to control Motor2 (back right)--> Motor[1][0]=6, Motor[1][1]=7
{8 , 9},   //input pin to control Motor3 (front left)--> Motor[2][0]=8, Motor[2][1]=9
{10, 11},  //input pin to control Motor4 (back left)--> Motor[3][0]=10, Motor[3][1] = 11
};

/* Motor enable pins */
#define EN1  9
#define EN2  3
#define EN3  12
#define EN4  13

/* motor commands */
#define STOP  'S'
#define FORWARD  'F'
#define BACKWARD  'B'
#define FRIGHT 'R'
#define FLEFT 'L'

/* motor command overhead */
const char SoP = 'C';
const char EoP = 'E';
const char nullTerminator = '\0';
unsigned char inByte;
#define MESSAGE_MAX_SIZE 5
char message[MESSAGE_MAX_SIZE];
char command;

#define F_CPU 16000000

int main(void)
{

    xTaskCreate(
    TaskTestTimers
    ,  (const portCHAR *)"TimerTest"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  5  // Priority (low num = low priority)
    ,  NULL );

    // xTaskCreate(
    // TaskMotorCommand
    // ,  (const portCHAR *)"MotorCommand"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  5  // Priority (low num = low priority)
    // ,  NULL );

    // xTaskCreate(
    // TaskIRSensorRead
    // ,  (const portCHAR *)"IRSensorRead"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  3  // Priority (low num = low priority)
    // ,  NULL );

    // xTaskCreate(
    // TaskIMURead
    // ,  (const portCHAR *)"IMURead"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  2  // Priority (low num = low priority)
    // ,  NULL );

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

void TaskTestTimers(void *pvParameters) {
    
    DDRC |= _BV(PC7) | _BV(PC6); //set DDC7 = 1 -> PC7 is output pin, DDC6 = 1 ->PC6 is output

    DDRD = 1;           /* make the D0 pin an output */   

    
    PORTC = 0x00;

    TCCR4A |= _BV(PWM4A) | _BV(WGM40) | _BV(COM4A1); //initiallize Timer 4 in freq/phase correct PWM mode
    TCCR4B = _BV(CS40); //initialize counter 4 with no prescaler

    TCCR3A |= _BV(WGM30) | _BV(COM3A1); //initiallize Timer 3 in 8 bit freq/phase correct PWM mode
    TCCR3B = _BV(CS31) ;//| _BV(CS30); //initialize counter 3 with prescaler 
    
    for(;;)
    {
        OCR3A = 0x7F;
        OCR4A = 0x7F; //set duty cycle into compare reg. TOP is 0xFF by default. 
        vTaskDelay(pdMS_TO_TICKS(100));
        PORTD ^= 1;    /* toggle the LED */
    }
}

void TaskMotorCommand( void *pvParameters)  // This is a Task.
{
    DDRD = 1;           /* make the D0 pin an output */
    for (;;) // A Task shall never return or exit. 
    {
        vTaskDelay(pdMS_TO_TICKS(100));
	    PORTD ^= 1;    /* toggle the LED */
    }
}

void TaskIRSensorRead(void *pvParameters) {

}

void TaskIMURead(void *pvParameters) {

}

void TaskSendSPIData(void *pvParameters) {

}

void TaskPIDController(void *pvParameters) {

}