#include <util/delay.h>

#include "Arduino_FreeRTOS.h"

#include "timing.h"
#include "PID_v1.h"

#include "motor.h"
#include "spi.h"
#include "ir.h"
#include "imu.h"
#include "pid.h"

volatile char spi_char ='\0';
volatile char spi_message[10];
volatile uint8_t pos = 0;
volatile uint8_t desired_heading_byte_num = 0;

// const char recv_index = 0;
const char start_byte = 's';
const char calib_req = 'v'; 
const char desired_heading = 'H';
const char linear_accel_req = 'X'; //r = I want roll
const char y_accel_req = 'y'; //p = I want pitch
const char heading_req = 'T'; //y = I want yaw
const char first_byte = '1';
const char second_byte = '2';
const char third_byte = '3';
const char fourth_byte = '4';
const char left_ir_req = 'l'; //< = I want left IR value
const char right_ir_req = 'r'; //> = I want right IR value
const char command_req = 'C'; // command is being sent
const char forward_req = 'f'; // move forward command
const char backward_req = 'B'; // move backward command
const char halt_req = 'h'; // halt command
const char rot_left_req = 'L'; // rotate right command
const char rot_right_req = 'R'; // rotate left command
const char stop_byte = 'e';
const char dummy_byte = '_';

const char ack_byte = '!';
const char err_byte = 'b';
const char ack_byte_stop = 'd'; 


// void vApplicationStackOverflowHook( TaskHandle_t xTask,
//                                     signed char *pcTaskName ) {
//     DDRD |= _BV(PD3);
//     PORTD |= _BV(PD3);
// }

void TaskIRSensorRead(void *pvParameters);

void TaskIMURead(void *pvParameters);

void TaskPIDController(void *pvParameters);

void TaskTestTimers(void *pvParameters);


#define F_CPU 16000000

#define BNO055_SAMPLERATE_DELAY_MS (100)


int main(void)
{
    DDRD |= _BV(PD2) | _BV(PD3);
    DDRE |= _BV(PE6); //debugging

    timing_init();

    init_spi();

    init_pwm();

    init_motor_pins();

    reset_speeds();

    xTaskCreate(
    TaskIRSensorRead
    ,  (const portCHAR *)"IRSensorRead"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority (low num = low priority)
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


void TaskIRSensorRead(void *pvParameters) {
    init_IR_pins();
    for(;;) {
        PORTD |= _BV(PD3);
        left_IR_read();
        right_IR_read();
        PORTD &= ~(_BV(PD3));
        if(IRrightVal > 90) {
            cli(); //disable interrupts
            stop_bot();
            sei(); //enable interrupts
        }
        else
        {}

        vTaskDelay(pdMS_TO_TICKS(500));

    }
}


void TaskIMURead(void *pvParameters) {
    init_imu();

    for(;;) {
        PORTD |= _BV(PD2);
        update_vals();
        PORTD &= ~(_BV(PD2));
        vTaskDelay(pdMS_TO_TICKS(BNO055_SAMPLERATE_DELAY_MS));
    }
}

void TaskPIDController(void *pvParameters) {

    init_pid();

    for(;;) {
        if(heading.num_float) {
            Input = (double) heading.num_float;
        }
        else {
            Input = 0;
        }
        myPID.Compute();

        speedFL += Output;
        speedBL += Output;
        speedFR -= Output;
        speedBR -= Output;

        limit_speeds();

        cli();
        update_speed();
        sei();
    }
    
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
    PORTE |= _BV(PE6); //debugging

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

    bool valid_start = spi_message[0] == start_byte;
    bool valid_command = false;
    bool valid_lin_accel = false;
    bool valid_y_accel = false;
    bool valid_heading = false;
    bool valid_desired_heading = false;
    if(pos >1) {
        valid_command = (valid_start && (spi_message[1] == command_req));
        valid_lin_accel = (valid_start && (spi_message[1] == linear_accel_req));
        valid_y_accel = (valid_start && (spi_message[1] == y_accel_req));
        valid_heading = (valid_start && (spi_message[1] == heading_req));
        valid_desired_heading = (valid_start && (spi_message[1] == desired_heading) && (pos < 7) && (pos > 2));
    }

    if(valid_desired_heading) {
        switch(desired_heading_byte_num) {
            case 0:
                desired_heading_num.bytes.first = spi_char;
                desired_heading_byte_num += 1;
                byte_to_send = spi_char;
                break;
            case 1:
                desired_heading_num.bytes.second = spi_char;
                desired_heading_byte_num += 1;
                byte_to_send = spi_char;
                break;
            case 2:
                desired_heading_num.bytes.third = spi_char;
                desired_heading_byte_num += 1;
                byte_to_send = spi_char;
                break;
            case 3:
                desired_heading_num.bytes.fourth = spi_char;
                desired_heading_byte_num = 0;
                byte_to_send = spi_char;
                break;
            default:
                desired_heading_byte_num = 0;
                byte_to_send = err_byte;
                break;

        }

    }
    else {
        switch(spi_char) {
            case start_byte:
                //start byte
                //send ack
                byte_to_send = ack_byte;
                break;
            case linear_accel_req:
                //linear acceleration value requested
                if(valid_start && pos == 2) {
                    byte_to_send  = sys_calib;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case y_accel_req: 
                //y acceleration value requested
                if(valid_start && pos == 2) {
                    byte_to_send  = sys_calib;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case heading_req: 
                //heading value requested
                if(valid_start && pos == 2) {
                    byte_to_send  = sys_calib;
                }  else {
                    byte_to_send = err_byte;
                } 
                break;
            case desired_heading:
                //about to receive desired heading
                if(valid_start && pos == 2) {
                    byte_to_send = ack_byte;
                } else {
                    byte_to_send = err_byte;
                }
                 break;
            case first_byte: 
                if(pos == 3 && valid_lin_accel) {
                    byte_to_send  = lin_accel.bytes.first;
                } else if(pos == 3 && valid_y_accel) {
                    byte_to_send = y_accel.bytes.first;
                } else if(pos == 3 && valid_heading) {
                    byte_to_send = heading.bytes.first;
                }
                else {
                    byte_to_send = err_byte;
                } 
                break;
            case second_byte: 
                if(pos == 4 && valid_lin_accel) {
                    byte_to_send  = lin_accel.bytes.second;
                } else if(pos == 4 && valid_y_accel) {
                    byte_to_send = y_accel.bytes.second;
                } else if(pos == 4 && valid_heading) {
                    byte_to_send = heading.bytes.second;
                }
                else {
                    byte_to_send = err_byte;
                } 
                break;
            case third_byte: 
                if(pos == 5 && valid_lin_accel) {
                    byte_to_send  = lin_accel.bytes.third;
                } else if(pos == 5 && valid_y_accel) {
                    byte_to_send = y_accel.bytes.third;
                } else if(pos == 5 && valid_heading) {
                    byte_to_send = heading.bytes.third;
                }
                else {
                    byte_to_send = err_byte;
                } 
                break;
            case fourth_byte: 
                if(pos == 6 && valid_lin_accel) {
                    byte_to_send  = lin_accel.bytes.fourth;
                } else if(pos == 6 && valid_y_accel) {
                    byte_to_send = y_accel.bytes.fourth;
                } else if(pos == 6 && valid_heading) {
                    byte_to_send = heading.bytes.fourth;
                }
                else {
                    byte_to_send = err_byte;
                } 
                break;
            case left_ir_req:
                //left ir sensor requested
                if(valid_start && pos == 2) {
                    byte_to_send  = (uint8_t) IRleftVal;
                }else {
                    byte_to_send = err_byte;
                }
                break;
            case right_ir_req:
                //left ir sensor requested
                if(valid_start && pos == 3) {
                    byte_to_send  = (uint8_t) IRrightVal;
                }else {
                    byte_to_send = err_byte;
                }
                break;
            case command_req:
                // master sending motor command
                // send ack byte back
                if (valid_start && pos == 2) {
                    byte_to_send = ack_byte;
                } else {
                    byte_to_send = err_byte;
                }
                break;
            case forward_req:
                // forward command
                // echo back command to comfirm to master
                if(valid_command && pos == 3) {
                    byte_to_send  = forward_req;
                    master_motor_command = FORWARD;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case backward_req:
                // forward command
                // echo back command to comfirm to master
                if(valid_command && pos == 3) {
                    byte_to_send  = backward_req;
                    master_motor_command = BACKWARD;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case halt_req:
                // halt command
                // echo back command to comfirm to master
                if(valid_command && pos == 3) {
                    byte_to_send  = halt_req;
                    master_motor_command = STOP;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case rot_left_req:
                // rotate left command
                // echo back command to comfirm to master
                if(valid_command && pos == 3) {
                    byte_to_send  = rot_left_req;
                    master_motor_command = LEFT;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case rot_right_req:
                // right command
                // echo back command to comfirm to master
                if(valid_command && pos == 3) {
                    byte_to_send  = rot_right_req;
                    master_motor_command = RIGHT;
                }  else {
                    byte_to_send = err_byte;
                }          
                break;
            case stop_byte:
                //send stop ack
                if ((valid_start && pos == 7)
                    || (valid_start && pos == 4))
                {
                    byte_to_send = ack_byte_stop;
                } else {
                    byte_to_send = err_byte;
                }
                break;
            case dummy_byte:
                //do nothing
                byte_to_send = dummy_byte;
                break;
            default:
                byte_to_send = err_byte;
                break;
        }
    }

    SPDR = byte_to_send;

    if (master_motor_command != '\0' && master_motor_command != command_state)
    {
        cli();
        stop_bot();
        delay(100);
        reset_speeds();
        moveRobot(master_motor_command);
        sei();
    }

    if(byte_to_send == err_byte || byte_to_send == dummy_byte){
        pos = 0;
    }

   PORTE &= ~(_BV(PE6));

      
}
