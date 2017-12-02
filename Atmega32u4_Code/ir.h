#ifndef IR_h
#define IR_h
#include <avr/io.h>


/* values read from IR sensor. Updated by IRSensorRead. Sent via SPI to XU4 */
extern int IRleftVal;
extern int IRrightVal;

void init_IR_pins(void);

#endif