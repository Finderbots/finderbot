#include "uart.h"
#include <avr/io.h>

void USART_Init(void)
{
    /* Set baud rate */
    UBRR1H = (unsigned char)(BAUD_PRESCALER>>8);
    UBRR1L = (unsigned char)BAUD_PRESCALER;

    /* Enable receiver and transmitter, enable receiver interrupt*/
    UCSR1B |= _BV(RXCIE1) | _BV(RXEN1)| _BV(TXEN1);

    /* Set frame format: 8data, 2stop bit */
    UCSR1C |= _BV(UPM11) | _BV(UCSZ11) | _BV(UCSZ10); //even parity
}


// function to send data on usart
void uart_transmit (unsigned char data)
{
    while (!( UCSR1A & (1<<UDRE1)));            // wait while register is free
    UDR1 = data;                             // load data in the register
}

// ISR(USART1_RX_vect)
// {
//     // Code to be executed when the USART receives a byte here
//     usart_rx_char = UDR1;

//     // if(usart_rx_char == 's') {
//     //     usart_tx_char = '!'; // Echo back the received byte back to the computer    
//     // }
//     // else {
//     //     usart_tx_char = '?';
//     // }

//     char master_motor_command = '\0';

//     // add to buffer if room
//     if (pos < (sizeof (uart_message) - 1)) {
//         uart_message[pos++] = usart_rx_char;
//     }
//     else
//     {
//         pos = 0;
//         uart_message[pos++] = usart_rx_char;
//     }

//     bool valid_start = false;
//     bool valid_command_msg = false;

//     if(pos >= 1)
//         valid_start = uart_message[0] == start_byte;
//     if(pos >=2)
//         valid_command_msg = (valid_start && (uart_message[1] == command_req));


//     switch(usart_rx_char) {
//         case start_byte:
//             //start byte
//             //send ack
//             usart_tx_char = ack_byte;
//             break;
//         case command_req:
//             // master sending motor command
//             // send ack byte back
//             if (valid_start && pos == 2) {
//                 usart_tx_char = ack_byte;
//             } else {
//                 usart_tx_char = err_byte;
//             }
//             break;
//         case forward_req:
//             // forward command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = forward_req;
//                 master_motor_command = FORWARD;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case backward_req:
//             // forward command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = backward_req;
//                 master_motor_command = BACKWARD;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case halt_req:
//             // halt command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = halt_req;
//                 master_motor_command = STOP;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case rot_left_req:
//             // rotate left command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = rot_left_req;
//                 master_motor_command = LEFT;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case rot_right_req:
//             // right command
//             // echo back command to comfirm to master
//             if(valid_command_msg && pos == 3) {
//                 usart_tx_char  = rot_right_req;
//                 master_motor_command = RIGHT;
//             }  else {
//                 usart_tx_char = err_byte;
//             }          
//             break;
//         case stop_byte:
//             //send ack
//             if (valid_command_msg && pos == 4)
//             {
//                 usart_tx_char = ack_byte_stop;
//             } else {
//                 usart_tx_char = err_byte;
//             }
//             break;
//         default:
//             usart_tx_char = err_byte;
//             break;
//     }

//     uart_transmit(usart_tx_char);

//     if (master_motor_command != '\0' && master_motor_command != command_state)
//     {
//         moveRobot(master_motor_command);
//     }

//     if(usart_tx_char == err_byte || usart_tx_char == ack_byte_stop){
//         pos = 0;
//     }


// }
