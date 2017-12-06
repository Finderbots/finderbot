#include <avr/io.h>
#include "motor.h"

/* speeds updated by PID and MotorCommand */
volatile int speedFL = 0;
volatile int speedFR = 0;
volatile int speedBL = 0;
volatile int speedBR = 0;

volatile char command_state = STOP;

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

void disable_motors(void) {
    TCCR4A &= ~(_BV(COM4A1)) & ~(_BV(COM4B1));
    TCCR4C &= ~(_BV(COM4D1));
    TCCR3A &=  ~(_BV(COM3A1));
}

void update_speed(void) {
    OCR4D = speedFR;
    OCR4B = speedBR;
    OCR4A = speedFL;
    OCR3A = speedBL;
}

void stop_bot(void) {
    disable_motors();
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