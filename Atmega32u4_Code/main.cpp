#include <util/delay.h>

#include "Arduino_FreeRTOS.h"

#include "timing.h"
#include "PID_v1.h"

#include "motor.h"
#include "spi.h"
#include "ir.h"
#include "imu.h"


// void vApplicationStackOverflowHook( TaskHandle_t xTask,
//                                     signed char *pcTaskName ) {
//     DDRD |= _BV(PD6);
//     PORTD ^= _BV(PD6);
// }

volatile char spi_char ='\0';
volatile char spi_message[MESSAGE_MAX_SIZE];
volatile uint8_t pos = 0;


 const char ack_byte = '!';
 const char recv_index = 0;
 const char start_byte = 's';
 const char info_req = 'i'; //i = information is being requested
 const char roll_req = 'r'; //r = I want roll
 const char pitch_req = 'p'; //p = I want pitch
 const char yaw_req = 'y'; //y = I want yaw
 const char left_ir_req = '<'; //< = I want left IR value
 const char right_ir_req = '>'; //> = I want right IR value
 const char command_req = 'C'; // command is being sent
 const char forward_req = 'f'; // move forward command
 const char backward_req = 'B'; // move backward command
 const char halt_req = 'h'; // halt command
 const char rot_left_req = 'L'; // rotate right command
 const char rot_right_req = 'R'; // rotate left command
 const char stop_byte = 'e';
 const char err_byte = 'b';
  const char ack_byte_stop = 'd'; 



void TaskMotorCommand(void *pvParameters);

void TaskIRSensorRead(void *pvParameters);

void updateAngle(void);

void TaskIMURead(void *pvParameters);

void TaskPIDController(void *pvParameters);

void TaskTestTimers(void *pvParameters);

double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

#define F_CPU 16000000

int main(void)
{
    timing_init();

    //initialize the variables we're linked to
    Input = yaw;
    Setpoint = 0;

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    DDRD |= _BV(PD3);

    // PORTB |= _BV(PB0);
    timing_init();
    // PORTB &= ~(_BV(PB0));

    DDRE |= _BV(PE6); //debugging

    init_spi();

    // while(1) {

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

    // xTaskCreate(
    // TaskIMURead
    // ,  (const portCHAR *)"IMURead"  // A name just for humans
    // ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  2  // Priority (low num = low priority)
    // ,  NULL );

    xTaskCreate(
    TaskPIDController
    ,  (const portCHAR *)"PIDController"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority (low num = low priority)
    ,  NULL );


    vTaskStartScheduler();

    return 0;               /* never reached */
}


void TaskIRSensorRead(void *pvParameters) {
    PORTD |= _BV(PD3);
    init_IR_pins();
    PORTD &= _BV(PD3);
    for(;;) {
        PORTE |= _BV(PE6);
        left_IR_read();
        right_IR_read();
        PORTE &= _BV(PE6);

        if(IRrightVal > 90) {
            //PORTE |= _BV(PE6); //debugging
            cli(); //disable interrupts
            stop_bot();
            sei(); //enable interrupts
        }
        else
        {
            //PORTE &= ~ _BV(PE6);
        }

     
    }
}

void TaskIMURead(void *pvParameters) {
    
    init_imu();
    
    for(;;) {
        
        updateAngle();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskPIDController(void *pvParameters) {
    
}

/*********ISRs******************************************/
//TImer0 ISR 
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

//ADC ISR
ISR(ADC_vect) 
{

    if(ADMUX & _BV(MUX0)) {
        IRrightVal = ADCH;
    }
    else  {
        IRleftVal = ADCH;
    }
}


// SPI Transmission/reception complete ISR
ISR(SPI_STC_vect)
{
    //PORTE |= _BV(PE6); //debugging

    char master_motor_command = '\0';

    spi_char = SPDR;  // grab byte from SPI Data Register
    
    if(spi_char == start_byte) {
        pos = 0;
    }

    // add to buffer if room
    if (pos < (sizeof (spi_message) - 1)) {
        spi_message[pos++] = spi_char;
    }
    else
    {
        pos = 0;
        spi_message[pos++] = spi_char;
    }

    uint8_t byte_to_send;

    switch(spi_char) {
        case start_byte:
            //start byte
            //send ack
            byte_to_send = ack_byte;
            break;
        case info_req: 
            //master requesting info byte
            //send ack
            if (spi_message[0] == start_byte && pos == 2) {
                byte_to_send = ack_byte;
            } else {
                byte_to_send = err_byte;
            }
            break;
        case roll_req:
            //roll value requested
            //send roll value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == info_req && pos == 3) {
                byte_to_send  = (uint8_t) roll;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case pitch_req: 
            //pitch value requested
            //send pitch value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == info_req && pos == 4) {
                byte_to_send  = (uint8_t) pitch;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case yaw_req: 
            //yaw value requested
            //send yaw value on spi line
            if(spi_message[0] == start_byte && spi_message[1] == info_req && pos == 5) {
                byte_to_send  = (uint8_t) yaw;
            }
            else {
                byte_to_send = err_byte;
            }
            break;
        case left_ir_req:
            //left ir sensor requested
            //send that value on the spi line
            if(spi_message[0] == start_byte && spi_message[1] == info_req && pos == 6) {
                byte_to_send  = (uint8_t) IRleftVal;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case right_ir_req:
            //left ir sensor requested
            //send that value on the spi line
            if(spi_message[0] == start_byte && spi_message[1] == info_req && pos == 7) {
                byte_to_send  = (uint8_t) IRrightVal;
            }else {
                byte_to_send = err_byte;
            }
            break;
        case command_req:
            // master sending motor command
            // send ack byte back
            if (spi_message[0] == start_byte && pos == 2) {
                byte_to_send = ack_byte;
            } else {
                byte_to_send = err_byte;
            }
            break;
        case forward_req:
            // forward command
            // echo back command to comfirm to master
            if(spi_message[0] == start_byte && spi_message[1] == command_req && pos == 3) {
                byte_to_send  = forward_req;
                master_motor_command = FORWARD;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case backward_req:
            // forward command
            // echo back command to comfirm to master
            if(spi_message[0] == start_byte && spi_message[1] == command_req && pos == 3) {
                byte_to_send  = backward_req;
                master_motor_command = BACKWARD;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case halt_req:
            // halt command
            // echo back command to comfirm to master
            if(spi_message[0] == start_byte && spi_message[1] == command_req && pos == 3) {
                byte_to_send  = halt_req;
                master_motor_command = STOP;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case rot_left_req:
            // rotate left command
            // echo back command to comfirm to master
            if(spi_message[0] == start_byte && spi_message[1] == command_req && pos == 3) {
                byte_to_send  = rot_left_req;
                master_motor_command = LEFT;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case rot_right_req:
            // right command
            // echo back command to comfirm to master
            if(spi_message[0] == start_byte && spi_message[1] == command_req && pos == 3) {
                byte_to_send  = rot_right_req;
                master_motor_command = RIGHT;
            }  else {
                byte_to_send = err_byte;
            }          
            break;
        case stop_byte:
            //send ack
            if (spi_message[0] == start_byte && ((spi_message[1] == info_req && pos == 8)
                || (spi_message[1] == command_req && pos == 4)))
            {
                byte_to_send = ack_byte_stop;
            } else {
                byte_to_send = err_byte;
            }
            break;
        default:
            byte_to_send = err_byte;
            break;
    }

    SPDR = byte_to_send;

    if (master_motor_command != '\0' && master_motor_command != command_state)
    {
        moveRobot(master_motor_command);
    }

    if(byte_to_send == err_byte || byte_to_send == ack_byte_stop){
        pos = 0;
    }

    PORTE &= ~(_BV(PE6));

      
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