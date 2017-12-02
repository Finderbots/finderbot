#include "spi.h"

#include <avr/io.h>

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