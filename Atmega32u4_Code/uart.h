#ifndef UART_h
#define UART_h

#include <inttypes.h>
#define UART_MESSAGE_MAX_SIZE 10

/*UART Variables*/
volatile uint8_t usart_rx_char;
volatile uint8_t usart_tx_char;
volatile char uart_message[UART_MESSAGE_MAX_SIZE];
volatile uint8_t pos;
const char ack_byte = '!';
const char recv_index = 0;
const char start_byte = 's';
const char command_req = 'C'; // command is being sent
const char forward_req = 'f'; // move forward command
const char backward_req = 'b'; // move backward command
const char halt_req = 'H'; // halt command
const char rot_left_req = 'l'; // rotate right command
const char rot_right_req = 'r'; // rotate left command
const char stop_byte = 'e';
const char err_byte = '?';
const char ack_byte_stop = 'd'; 


#define USART_BAUD 57600
#define BAUD_PRESCALER (F_CPU/(16UL*USART_BAUD)-1)

void USART_Init(void);

#endif