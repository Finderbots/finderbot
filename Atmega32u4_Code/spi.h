#ifndef SPI_h
#define SPI_h

#include <inttypes.h>

// /*SPI Variables*/
#define MESSAGE_MAX_SIZE 10

extern volatile char spi_char;
extern volatile char spi_message[MESSAGE_MAX_SIZE];
extern volatile uint8_t pos;

void init_spi(void);

#endif 