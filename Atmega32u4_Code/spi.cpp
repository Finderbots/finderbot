#include "spi.h"

#include <avr/io.h>


void init_spi(void)
{
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