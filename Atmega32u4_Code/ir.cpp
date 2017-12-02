#include "ir.h"

/* values read from IR sensor. Updated by IRSensorRead. Sent via SPI to XU4 */
int IRleftVal = 0;
int IRrightVal = 0;


void init_IR_pins(void) {
   // DDRE |= _BV(PE6); //debugging
   // DDRB |= _BV(PB0); //debugging
   // DDRD |= _BV(PD2) | _BV(PD3); //debugging
    ADCSRA |= _BV(ADEN) | _BV(ADIE); //enable ADC
    ADMUX |= _BV(REFS0) | _BV(ADLAR) ; //left shift adc result register
}