#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>

#include "Arduino_FreeRTOS.h"

#include "timing.h"
#include "PID_v1.h"

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

void moveRobot(char command);

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
//char message[MESSAGE_MAX_SIZE];
volatile char command_state;

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
float Kp = 0.5;
float Ki = 0.05;
float Kd = 0.4;

float offsetLoc = 0;
float pT,iT,dT = 0;
float errorS = 0;
float prevE = 0;

// /*SPI Variables*/
// volatile char spi_char;
// volatile char spi_message[MESSAGE_MAX_SIZE];
// volatile byte pos;
// const char ack_byte = '!';
// const char recv_index = 0;
// const char start_byte = 's';
// const char info_req = 'i'; //i = information is being requested
// const char roll_req = 'r'; //r = I want roll
// const char pitch_req = 'p'; //p = I want pitch
// const char yaw_req = 'y'; //y = I want yaw
// const char left_ir_req = '<'; //< = I want left IR value
// const char right_ir_req = '>'; //> = I want right IR value
// const char command_req = 'C'; // command is being sent
// const char forward_req = 'f'; // move forward command
// const char backward_req = 'B'; // move backward command
// const char halt_req = 'h'; // halt command
// const char rot_left_req = 'L'; // rotate right command
// const char rot_right_req = 'R'; // rotate left command
// const char stop_byte = 'e';
// const char err_byte = 'b';
// const char ack_byte_stop = 'd'; 



/*UART Variables*/
volatile uint8_t usart_rx_char;
volatile uint8_t usart_tx_char;
volatile char uart_message[MESSAGE_MAX_SIZE];
volatile byte pos;
const char ack_byte = '!';
const char recv_index = 0;
const char start_byte = 's';
const char command_req = 'C'; // command is being sent
const char forward_req = 'f'; // move forward command
const char backward_req = 'b'; // move backward command
const char halt_req = 'H'; // halt command
const char rot_left_req = 'l'; // rotate right command
const char rot_right_req = 'r'; // rotate left command
const char stop_byte = 'e';
const char err_byte = '?';
const char ack_byte_stop = 'd'; 

#define F_CPU 16000000

#define USART_BAUD 57600
#define BAUD_PRESCALER (F_CPU/(16UL*USART_BAUD)-1)

void USART_Init(void)
{
    /* Set baud rate */
    UBRR1H = (unsigned char)(BAUD_PRESCALER>>8);
    UBRR1L = (unsigned char)BAUD_PRESCALER;

    /* Enable receiver and transmitter, enable receiver interrupt*/
    UCSR1B |= _BV(RXCIE1) | _BV(RXEN1)| _BV(TXEN1);

    /* Set frame format: 8data, 2stop bit */
    UCSR1C |= _BV(UPM11) | _BV(UCSZ11) | _BV(UCSZ10); //even parity
}


// function to send data on usart
void uart_transmit (unsigned char data)
{
    while (!( UCSR1A & (1<<UDRE1)));            // wait while register is free
    UDR1 = data;                             // load data in the register
}

// ISR(USART1_RX_vect)
// {
//     // Code to be executed when the USART receives a byte here
//     usart_rx_char = UDR1;

//     // if(usart_rx_char == 's') {
//     //     usart_tx_char = '!'; // Echo back the received byte back to the computer    
//     // }
//     // else {
//     //     usart_tx_char = '?';
//     // }

//     char master_motor_command = '\0';

//     // add to buffer if room
//     if (pos < (sizeof (uart_message) - 1)) {
//         uart_message[pos++] = usart_rx_char;
//     }
//     else
//     {
//         pos = 0;
//         uart_message[pos++] = usart_rx_char;
//     }

//     bool valid_start = false;
//     bool valid_command_msg = false;

//     if(pos >= 1)
//         valid_start = uart_message[0] == start_byte;
//     if(pos >=2)
//         valid_command_msg = (valid_start && (uart_message[1] == command_req));


//     switch(usart_rx_char) {
//         case start_byte:
//             //start byte
//             //send ack
//             usart_tx_char = ack_byte;
//             break;
//         case command_req:
//             // master sending motor command
//             // send ack byte back
//             if (valid_start && pos == 2) {
//                 usart_tx_char = ack_byte;
//             } else {
//                 usart_tx_char = err_byte;
//             }
//             break;
//         case forward_req:
//             // forward command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = forward_req;
//                 master_motor_command = FORWARD;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case backward_req:
//             // forward command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = backward_req;
//                 master_motor_command = BACKWARD;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case halt_req:
//             // halt command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = halt_req;
//                 master_motor_command = STOP;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case rot_left_req:
//             // rotate left command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = rot_left_req;
//                 master_motor_command = LEFT;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case rot_right_req:
//             // right command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = rot_right_req;
//                 master_motor_command = RIGHT;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case stop_byte:
//             //send ack
//             if (valid_command_msg && pos == 4)
//             {
//                 usart_tx_char = ack_byte_stop;
//             } else {
//                 usart_tx_char = err_byte;
//             }
//             break;
//         default:
//             usart_tx_char = err_byte;
//             break;
//     }

//     uart_transmit(usart_tx_char);

//     if (master_motor_command != '\0' && master_motor_command != command_state)
//     {
//         moveRobot(master_motor_command);
//     }

//     if(usart_tx_char == err_byte || usart_tx_char == ack_byte_stop){
//         pos = 0;
//     }


// }

ISR(TIMER0_OVF_vect)
{
  // copy these to local variables so they can be stored in registers
  // (volatile variables must be read from memory on every access)
  unsigned long m = timer0_millis;
  unsigned char f = timer0_fract;

  m += MILLIS_INC;
  f += FRACT_INC;
  if (f >= FRACT_MAX) {
    f -= FRACT_MAX;
    m += 1;
  }

  timer0_fract = f;
  timer0_millis = m;
  timer0_overflow_count++;
}


int main(void)
{

    DDRB |= _BV(PB0);

    // PORTB |= _BV(PB0);
    // timing_init();
    // PORTB &= ~(_BV(PB0));

    DDRE |= _BV(PE6); //debugging

    // cli(); //disable interrupts while initializing usart
    // USART_Init();
    // sei();

    // while(1) {
    //     uart_transmit('t');
    //     delay(1000);
    // }


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

    xTaskCreate(
    TaskIMURead
    ,  (const portCHAR *)"IMURead"  // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority (low num = low priority)
    ,  NULL );

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
    command_state = command;

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
            command_state = STOP;
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
    // prevAngles[prevAngleI] = angles[1];
    // prevAngleI = (prevAngleI + 1) % AvgAngles;
    // float sum = 0;
    // for (int i = 0; i < AvgAngles; i++)
    //     sum += prevAngles[i];
    // currAngle = sum / AvgAngles;
    // prevAngle = currAngle;
    //PORTD &= ~(_BV(PD2)); //debugging
}

void TaskIMURead(void *pvParameters) {
    
    // Wire.begin();
    // //PORTB |= _BV(PB0);

    // vTaskDelay(1); //1 tick
    // sixDOF.init(); //Begin the IMU
    // //PORTB &= ~(_BV(PB0));
    // vTaskDelay(1); //1 tick 
    PORTE |= _BV(PE6);

    for(;;) {
        PORTB ^= _BV(PB0);
        //updateAngle();

        //uart_transmit('t');
        //PORTB &= ~(_BV(PB0));
        vTaskDelay(pdMS_TO_TICKS(1000));
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