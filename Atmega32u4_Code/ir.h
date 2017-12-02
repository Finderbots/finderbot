#ifndef IR_h
#define IR_h
#include <avr/io.h>
#include <avr/sleep.h>

/* values read from IR sensor. Updated by IRSensorRead. Sent via SPI to XU4 */
extern int IRleftVal;
extern int IRrightVal;

void init_IR_pins(void);
void right_IR_read();
void left_IR_read();

#endif