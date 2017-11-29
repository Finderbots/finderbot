#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "Arduino_FreeRTOS.h"

#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "FreeSixIMU.h"

// void vApplicationStackOverflowHook( TaskHandle_t xTask,
//                                     signed char *pcTaskName ) {
//     DDRD |= _BV(PD6);
//     PORTD ^= _BV(PD6);
// }

void init_pwm();

void init_spi(void);

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
uint8_t roll = 0;
uint8_t pitch = 0;
uint8_t yaw = 0;

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
#define MESSAGE_MAX_SIZE 10
char message[MESSAGE_MAX_SIZE];
char command;

/*IMU constants & objects*/ 

FreeSixIMU sixDOF = FreeSixIMU();

//TwoWire Wire = TwoWire();

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

/*SPI Variables*/
volatile char spi_char;
volatile char spi_message[MESSAGE_MAX_SIZE];
volatile byte pos;
const char ack_byte = '!';
const char recv_index = 0;
const char start_byte = 's';
const char request = 'i'; //i = information is being requested
const char roll_req = 'r'; //r = I want roll
const char pitch_req = 'p'; //p = I want pitch
const char yaw_req = 'y'; //y = I want yaw
const char left_ir_req = '<'; //< = I want left IR value
const char right_ir_req = '>'; //> = I want right IR value
const char stop_byte = 'e';
const char err_byte = 'b';
const char ack_byte_stop = 'd'; 



#define F_CPU 16000000

int main(void)
{
    //DDRD |= _BV(PD3); //debug
    //PORTD ^= _BV(PD3); //debug

    init_spi();

    sei();
    //SREG |= (1<<7);
    volatile int i = 0;
    volatile int j = 0;

    while(1) {
        i++;
        j = i;
    }

    // xTaskCreate(
    // TaskTestTimers
    // ,  (const portCHAR *)"TimerTest"  // A name just for humans
    // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  5  // Priority (low num = low priority)
    // ,  NULL );

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
    // ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
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
    DDRD |= _BV(PD7); //set DDD7 as output -> PD7
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

void init_spi(void)
{
    DDRE |= _BV(PE6); //debugging

    SPCR |= _BV(SPE); //enable spi mode

    SPCR &= ~(_BV(MSTR)); //explicitly set spi to slave mode

    DDRB &= ~(_BV(PB0)); //set SS as input explicitly 

    // Set MISO output, all others input 
    DDRB |= _BV(PB3);

    // Enable spi interrupts 
    SPCR |= _BV(SPIE); 

    //might need to set _BV(CPOL)(Clk polarity)
                                // and _BV(CPHA) (clk phase) 
                                //_BV(DORD) (data order) to match master(ODROID)
}

// SPI Transmission/reception complete ISR
ISR(SPI_STC_vect)
{
    PORTE |= _BV(PE6); //debugging

    spi_char = SPDR;  // grab byte from SPI Data Register
    
    if(spi_char == start_byte) {
        pos = 0;
    }

    // add to buffer if room
    if (pos < (sizeof (spi_message) - 1)) {
        spi_message[pos++] = spi_char;
    }

    uint8_t byte_to_send;

    switch(spi_char) {
        case start_byte:
            //start byte
            //send ack
            byte_to_send = ack_byte;
            break;
        case request: 
            //master requesting info byte
            //send ack
            byte_to_send = ack_byte;
            break;
        case roll_req:
            //roll value requested
            //send roll value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == request) {
                byte_to_send  = (uint8_t) roll;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case pitch_req: 
            //pitch value requested
            //send pitch value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == request) {
                byte_to_send  = (uint8_t) pitch;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case yaw_req: 
            //yaw value requested
            //send yaw value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == request) {
                byte_to_send  = (uint8_t) yaw;
            }
            else {
                byte_to_send = err_byte;
            }
            break;
        case left_ir_req:
            //left ir sensor requested
            //send that value on the spi line
            if(spi_message[0] == start_byte && spi_message[1] == request) {
                byte_to_send  = (uint8_t) IRleftVal;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case right_ir_req:
            //left ir sensor requested
            //send that value on the spi line
            if(spi_message[0] == start_byte && spi_message[1] == request) {
                byte_to_send  = (uint8_t) IRrightVal;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case stop_byte:
            //send ack
            byte_to_send = ack_byte_stop;
            break;
        default:
            byte_to_send = err_byte;
            break;
    }

    SPDR = byte_to_send;

    if(byte_to_send == err_byte || byte_to_send == ack_byte_stop){
        pos = 0;
    }

    PORTE &= ~(_BV(PE6));

      
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

ISR(ADC_vect) 
{

    if(ADMUX & _BV(MUX0)) {
        IRrightVal = ADCH;
    }
    else  {
        IRleftVal = ADCH;
    }
}



void init_IR_pins(void) {
   // DDRE |= _BV(PE6); //debugging
   // DDRB |= _BV(PB0); //debugging
   // DDRD |= _BV(PD2) | _BV(PD3); //debugging
    ADCSRA |= _BV(ADEN) | _BV(ADIE); //enable ADC
    ADMUX |= _BV(REFS0) | _BV(ADLAR) ; //left shift adc result register
}

void TaskIRSensorRead(void *pvParameters) {
    init_IR_pins();

    for(;;) {

        /*******left ir sensor*********/
        ADMUX &= ~(_BV(MUX4)) & ~(_BV(MUX3)) & ~(_BV(MUX2))& ~(_BV(MUX1)) & ~(_BV(MUX0)); //choose channel_0
        ADCSRB &= ~(_BV(MUX5));

        //ADCSRA |= _BV(ADSC); //start single conversion

        
        SMCR |= _BV(SM0); //enter adc noise reduction (sleep) mode
        SMCR |= _BV(SE); //enter sleep mode
        sleep_cpu();
        SMCR &= ~(_BV(SE)); //disable sleep mode

        // while(!(ADCSRA & _BV(ADIF))){ //wait for conversion to finish - finished when ADIF is set to 1
        //     continue;
        // }
        // //PORTB ^= _BV(PB0); debugging
        // //IRleftVal = ADCL;
        // IRleftVal = ADCH; //only need 8 bit precision

        // ADCSRA |= _BV(ADIF); //clear the interrupt flag maybe??? datasheet says to write a one to it to clear it...


        /*******right ir sensor*********/
        ADMUX &= ~(_BV(MUX4)) & ~(_BV(MUX3)) & ~(_BV(MUX2))& ~(_BV(MUX1)); //choose channel_1
        ADCSRB &= ~(_BV(MUX5)); //choose channel_1
        
        ADMUX |= _BV(MUX0); //choose channel_1

        PORTB &= ~(_BV(PB0));
        // ADCSRA |= _BV(ADSC); //start single conversion
        
        SMCR |= _BV(SM0); //enter adc noise reduction (sleep) mode
        SMCR |= _BV(SE); //enter sleep mode
        sleep_cpu();
        SMCR &= ~(_BV(SE)); //disable sleep mode

        // while(!(ADCSRA & _BV(ADIF))){ //wait for conversion to finish - finished when ADIF is set to 1
        //     continue;
        // }
        PORTB |= _BV(PB0); //debugging

        //IRrightVal = ADCH; //only need 8 bit precision

        //ADCSRA |= _BV(ADIF); //clear the interrupt flag maybe??? datasheet says to write a one to it to clear it...

        if(IRrightVal > 90) {
            PORTE |= _BV(PE6); //debugging
            cli(); //disable interrupts
            stop_bot();
            sei(); //enable interrupts
        }
        else
        {
            PORTE &= ~ _BV(PE6);
        }

        // //testing
        // if(IRrightVal <20) {
        //     PORTB ^= _BV(PB0);
        // }
        // if(IRrightVal <40){
        //     PORTE ^= _BV(PE6);
        // }
        // if(IRrightVal <60) {
        //     PORTD ^= _BV(PD2);
        // }
        // if(IRrightVal <80) {
        //     PORTD ^= _BV(PD3);
        // }

        //vTaskDelayUntil(TickType_t *const pxPreviousWakeTime, const TickType_t xTimeIncrement)
    }
}


void updateAngle(void) {
    //PORTD |= _BV(PD2); //debugging
    sixDOF.getYawPitchRoll(angles);
    prevAngles[prevAngleI] = angles[1];
    prevAngleI = (prevAngleI + 1) % AvgAngles;
    float sum = 0;
    for (int i = 0; i < AvgAngles; i++)
        sum += prevAngles[i];
    currAngle = sum / AvgAngles;
    prevAngle = currAngle;
    //PORTD &= ~(_BV(PD2)); //debugging
}

void TaskIMURead(void *pvParameters) {
    Wire.begin();

    vTaskDelay(1); //1 tick
    sixDOF.init(); //Begin the IMU
    vTaskDelay(1); //1 tick

    DDRD |= _BV(PD2) | _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6);
 

    for(;;) {
        updateAngle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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