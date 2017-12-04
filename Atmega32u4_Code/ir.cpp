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

void left_IR_read() {
    /*******left ir sensor*********/
    ADMUX &= ~(_BV(MUX0)); //choose channel_0

    SMCR |= _BV(SM0); //enter adc noise reduction (sleep) mode
    SMCR |= _BV(SE); //enable sleep mode
    sleep_cpu();
    SMCR &= ~(_BV(SE)); //disable sleep mode
}

void right_IR_read() {
      /*******right ir sensor*********/
    ADMUX |= _BV(MUX0); //choose channel_1

    SMCR |= _BV(SM0); //enter adc noise reduction (sleep) mode
    SMCR |= _BV(SE); //enter sleep mode
    sleep_cpu();
    SMCR &= ~(_BV(SE)); //disable sleep mode
}