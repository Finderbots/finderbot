#ifndef DATA_TRANSMISSION_H
#define DATA_TRANSMISSION_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "wifi_connect.h"

// port values for each type for data transmission
#define BASE_PORT_MAP 8001
#define BASE_PORT_AUDIO_OUT 8002
#define BASE_PORT_AUDIO_IN 8003
#define BASE_PORT_IMAGE 8004

#define ROBOT_PORT_MAP 9001
#define ROBOT_PORT_AUDIO_IN 9002
#define ROBOT_PORT_AUDIO_OUT 9003
#define ROBOT_PORT_IMAGE 9004

// sends data over given socket connection. only for tcp.
void send_data(int sock, char* msg, int len);

// receives data over given socket connection. only for tcp.
int recv_data(int sock, char* buf, int max_len);

/* probably gonna cut and just send map image.
// creates socket and connection for send map data back to base station. 
int base_init_map();

// creates socket and connection to receive map data back to base station. 
int robot_init_map();

// sends occupancy grid to base over connection
int send_map_from_robot(char** byte_map, int num_rows, int num_cols);

// receives occupancy grid from robot over connection 
int recv_map_at_base();
*/

// creates udp socket for receiving audio at the base from the robot
int base_init_audio_in();

// creates udp socket for sending audio from the robot to the base
int robot_init_audio_out();

// sends audio data from the robot to the base via udp socket 
int robot_send_audio();

// receives audio data at the base from the robot via udp socket
void base_recv_audio();

// creates udp socket for sending audio from the base to the robot
int base_init_audio_out();

// creates udp socket for receiving audio at the robot from the base 
int robot_init_audio_in();

// sends audio data from the base to the robot via udp socket
void base_send_audio();

// receives audio data at the robot from the basevia udp socket
void robot_recv_audio();

// creates udp socket for receiving images at the base from the robot
int base_init_image();

// creates udp socket for sending images from the robot to the base
int robot_init_image();

// sends the image at the given path over sevaral packets from the robot to the base
void send_image_from_robot(const char* image_path);

// receives the image at the base from the robot
void recv_image_at_base();

#endif /* DATA_TRANSMISSION_H */