#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdint.h>
#include <alsa/asoundlib.h>

#include "data_transmission.h"
#include "wifi_connect.h"
#include "../audio/audio_interface.h"

int base_sock_map;
int robot_sock_map;
int base_sock_audio_out;
int robot_sock_audio_in;
int base_sock_audio_in;
int robot_sock_audio_out;
int base_sock_image;
int robot_sock_image;

struct sockaddr_in base_audio_out_addr;
struct sockaddr_in robot_audio_in_addr;
struct sockaddr_in base_audio_in_addr;
struct sockaddr_in robot_audio_out_addr;
struct sockaddr_in base_image_addr;
struct sockaddr_in robot_image_addr;

void send_data(int sock, char* msg, int len)
{
	int bytes_sent = 0;

	while (len > 0)
	{
		bytes_sent = send(sock, msg, len, 0);

		if (bytes_sent == -1)
		{
			printf("send failed\n");
			exit(1);
		}

		len = len - bytes_sent;
		msg = msg + bytes_sent;
	}
}

int recv_data(int sock, char* buf, int max_len)
{
	int ret = recv(sock, buf, max_len, 0);
	if (ret == 0)
	{
		printf("connection closed\n");
		exit(1);
	}
	else if (ret == -1)
	{
		printf("recv error\n");
		exit(1);
	}

	return ret;
}

/*
int base_init_map()
{
	int connect_sock;
	struct sockaddr_in address;
	int addrlen = sizeof(address);
	int opt = 1;
	  
	// Creating socket file descriptor
	if ((connect_sock = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		printf("socket failed\n");
		return -1;
	}

	if (setsockopt(connect_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		printf("setsockopt failed\n");
		return -1;
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(IP_BASE);
	address.sin_port = htons(BASE_PORT_MAP);
	  
	// Forcefully attaching socket to PORT_MAP
	if (bind(connect_sock, (struct sockaddr *)&address, sizeof(address))<0)
	{
		printf("bind failed\n");
		return -1;
	}
	if (listen(connect_sock, 3) < 0)
	{
		printf("listen failed\n");
		return -1;
	}
	if ((base_sock_map = accept(connect_sock, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
	{
		printf("accept failed\n");
		return -1;
	}

	return 0;
}

int robot_init_map()
{
	struct sockaddr_in address;
	struct sockaddr_in serv_addr;

	if ((robot_sock_map = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("Socket creation error \n");
		return -1;
	}
  
	memset(&serv_addr, '0', sizeof(serv_addr));
  
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(BASE_PORT_MAP);
	  
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_BASE, &serv_addr.sin_addr)<=0) 
	{
		printf("Invalid address/ Address not supported \n");
		return -1;
	}
  
	if (connect(robot_sock_map, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("Connection Failed \n");
		return -1;
	}

	return 0;
}

int send_map_from_robot(char** byte_map, int num_rows, int num_cols)
{
	//send map size information to base station
	char size_info[16];
	sprintf(size_info, "%i %i ", num_rows, num_cols);
	send_data(robot_sock_map, size_info, strlen(size_info));

	// send map data row by row
	for (int row = 0; row < num_rows; ++row)
	{
		send_data(robot_sock_map, byte_map[row], num_cols);
		sleep(1);
	}
	
	return 0;
}

int recv_map_at_base()
{
	// receive map data
	int max_len = 1024;
	char buf[max_len]; 

	recv_data(base_sock_map, buf, max_len);

	int space_loc = -1;
	for (int i = 0; i < max_len; ++i)
	{
		if (buf[i] == ' ')
		{
			space_loc = i;
			break;
		}
	}
	if (space_loc == -1)
	{
		printf("improper size info received\n");
		exit(1);
	}

	int num_rows = atoi(buf);
	int num_cols = atoi(buf + space_loc);

	// print for testing
	printf("num_rows: %i\nnum_cols: %i\n\n", num_rows, num_cols);

	char map_data[num_rows][num_cols];

	for (int row = 0; row < num_rows; ++row)
	{
		int tot_bytes_recv = 0;
		while (tot_bytes_recv < num_cols)
		{
			int bytes_recv = recv_data(base_sock_map, map_data[row]+tot_bytes_recv, 
				num_cols-tot_bytes_recv);
			tot_bytes_recv = tot_bytes_recv + bytes_recv;
		}
	}

	// print for testing
	for (int row = 0; row < num_rows; ++row)
	{
		for (int col = 0; col < num_cols; ++ col)
		{
			printf("%i ", map_data[row][col]);
		}
		printf("\n");
	}

	// TODO:
	// come up with effective way to return data. probably struct, etc. but more info needed from Jon.
	return 0;
}
*/

int base_init_audio_in()
{
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        int opt = 1;

        // Creating socket file descriptor
        if ((base_sock_audio_in = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
        {
                printf("socket failed\n");
                return -1;
        }

        if (setsockopt(base_sock_audio_in, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
                printf("setsockopt failed\n");
                return -1;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr(IP_BASE);
        address.sin_port = htons(BASE_PORT_AUDIO_IN);

        // Forcefully attaching socket to PORT_MAP
        if (bind(base_sock_audio_in, (struct sockaddr *)&address, sizeof(address))<0)
        {
                printf("bind failed\n");
                return -1;
        }

        // set up for recvfrom
        addrlen = sizeof(robot_audio_out_addr);
        memset(&robot_audio_out_addr, '0', sizeof(robot_audio_out_addr));

        robot_audio_out_addr.sin_family = AF_INET;
        robot_audio_out_addr.sin_port = htons(ROBOT_PORT_AUDIO_OUT);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET, IP_BASE, &robot_audio_out_addr.sin_addr)<=0)
        {
                printf("Invalid address/ Address not supported \n");
                return -1;
        }

        setup_speaker_base();
}


int base_init_audio_out()
{
	if ((base_sock_audio_out = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("Socket creation error \n");
		return -1;
	}
  
	memset(&robot_audio_in_addr, '0', sizeof(robot_audio_in_addr));
  
	robot_audio_in_addr.sin_family = AF_INET;
	robot_audio_in_addr.sin_port = htons(ROBOT_PORT_AUDIO_IN);
	  
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_ROBOT, &robot_audio_in_addr.sin_addr)<=0) 
	{
		printf("Invalid address/ Address not supported \n");
		return -1;
	}

	setup_mic_base();

	return 0;
}

int robot_init_audio_in()
{
	struct sockaddr_in address;
	int addrlen = sizeof(address);
	int opt = 1;
	  
	// Creating socket file descriptor
	if ((robot_sock_audio_in = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
	{
		printf("socket failed\n");
		return -1;
	}

	if (setsockopt(robot_sock_audio_in, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		printf("setsockopt failed\n");
		return -1;
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(IP_ROBOT);
	address.sin_port = htons(ROBOT_PORT_AUDIO_IN);
	  
	// Forcefully attaching socket to PORT_MAP
	if (bind(robot_sock_audio_in, (struct sockaddr *)&address, sizeof(address))<0)
	{
		printf("bind failed\n");
		return -1;
	}

	// set up for recvfrom
	addrlen = sizeof(base_audio_out_addr);
	memset(&base_audio_out_addr, '0', sizeof(base_audio_out_addr));
  
	base_audio_out_addr.sin_family = AF_INET;
	base_audio_out_addr.sin_port = htons(BASE_PORT_AUDIO_OUT);
	  
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_BASE, &base_audio_out_addr.sin_addr)<=0) 
	{
		printf("Invalid address/ Address not supported \n");
		return -1;
	}

	setup_speaker_robot();
}

int robot_init_audio_out()
{
        if ((robot_sock_audio_out = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
                printf("Socket creation error \n");
                return -1;
        }

        memset(&base_audio_in_addr, '0', sizeof(base_audio_in_addr));

        base_audio_in_addr.sin_family = AF_INET;
        base_audio_in_addr.sin_port = htons(BASE_PORT_AUDIO_IN);
          
        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET, IP_ROBOT, &base_audio_in_addr.sin_addr)<=0)
        {
                printf("Invalid address/ Address not supported \n");
                return -1;
        }

        setup_mic_robot();

        return 0;
}

void base_send_audio()
{
	int buf_frame = 128;
	int buf_len = buf_frame * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * 2;
	uint16_t buf[buf_len];
	int err = 0;
	
	while (1)
	{
		base_capture(buf, buf_frame);

		if (sendto(base_sock_audio_out, buf, buf_len, 0, (struct sockaddr *) &robot_audio_in_addr, sizeof(robot_audio_in_addr)) < 0) 
		{
			printf("ERROR in sendto");
			exit(1);
		}
	}
}

void robot_send_audio()
{
        int buf_frame = 128;
        int buf_len = buf_frame * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * 2;
        uint16_t buf[buf_len];
        int err = 0;

        while (1)
        {
                robot_capture(buf, buf_frame);

                if (sendto(robot_sock_audio_out, buf, buf_len, 0, (struct sockaddr *) &base_audio_in_addr, sizeof(base_audio_in_addr)) < 0)
                {
                        printf("ERROR in sendto");
                        exit(1);
                }
        }
}

void base_recv_audio()
{
        int buf_frame = 128;
        int buf_len = buf_frame * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * 2;
        uint16_t buf[buf_len];
        int err = 0;
        int robot_audio_out_addr_len = sizeof(robot_audio_out_addr);

        while (1)
        {
                // recv audio
                memset(buf, 0, buf_len);
                int bytes_recv = recvfrom(base_sock_audio_in, buf, buf_len, 0, (struct sockaddr *) &robot_audio_out_addr, &robot_audio_out_addr_len);
                if (bytes_recv <= 0)
                {
                        printf("ERROR in recvfrom");
                        exit(1);
                }

                base_playback(buf, buf_frame);
        }
}


void robot_recv_audio()
{
	int buf_frame = 128;
	int buf_len = buf_frame * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * 2;
	uint16_t buf[buf_len];
	int err = 0;
	int base_audio_out_addr_len = sizeof(base_audio_out_addr);
	
	while (1) 
	{
		// recv audio
		memset(buf, 0, buf_len);
		int bytes_recv = recvfrom(robot_sock_audio_in, buf, buf_len, 0, (struct sockaddr *) &base_audio_out_addr, &base_audio_out_addr_len);
		if (bytes_recv <= 0)
		{
			printf("ERROR in recvfrom");
			exit(1);
		}

		robot_playback(buf, buf_frame);
	}
}

int base_init_image()
{
	struct sockaddr_in address;
	int addrlen = sizeof(address);
	int opt = 1;
	  
	// Creating socket file descriptor
	if ((base_sock_image = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
	{
		printf("socket failed\n");
		return -1;
	}

	if (setsockopt(base_sock_image, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		printf("setsockopt failed\n");
		return -1;
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(IP_BASE);
	address.sin_port = htons(BASE_PORT_IMAGE);
	  
	// Forcefully attaching socket to PORT_MAP
	if (bind(base_sock_image, (struct sockaddr *)&address, sizeof(address))<0)
	{
		printf("bind failed\n");
		return -1;
	}

	// set up for recvfrom
	addrlen = sizeof(robot_image_addr);
	memset(&robot_image_addr, '0', sizeof(robot_image_addr));
  
	robot_image_addr.sin_family = AF_INET;
	robot_image_addr.sin_port = htons(ROBOT_PORT_IMAGE);
	  
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_ROBOT, &robot_image_addr.sin_addr)<=0) 
	{
		printf("Invalid address/ Address not supported \n");
		return -1;
	}

	return 0;
}

int robot_init_image()
{
	if ((robot_sock_image = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("Socket creation error \n");
		return -1;
	}
  
	memset(&base_image_addr, '0', sizeof(base_image_addr));
  
	base_image_addr.sin_family = AF_INET;
	base_image_addr.sin_port = htons(BASE_PORT_IMAGE);
	  
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_BASE, &base_image_addr.sin_addr)<=0) 
	{
		printf("Invalid address/ Address not supported \n");
		return -1;
	}

	return 0;
}

void send_image_from_robot(const char* image_path)
{
	char buf[1024];

	// get file len
	FILE* f_ptr;
	int file_len;
	f_ptr = fopen(image_path, "rb");
	fseek(f_ptr, 0, SEEK_END);
	file_len = ftell(f_ptr);
	rewind(f_ptr);


	// send start message
	memset(buf, 0, 1024);
	sprintf(buf, "start %i %i", file_len, 1024);
	int buf_len = strlen(buf);
	if (sendto(robot_sock_image, buf, buf_len, 0, (struct sockaddr *) &base_image_addr, sizeof(base_image_addr)) < 0) 
	{
		printf("ERROR in sendto");
		exit(1);
	}

	// send file data
	for (int bytes = 0; bytes < file_len; bytes = bytes + 1024)
	{
		memset(buf, 0, 1024);
		buf_len = 1024;
		if (bytes + 1024 > file_len) 
		{
			buf_len = file_len - bytes;
		}
		fread(buf, sizeof(char), buf_len, f_ptr);
		
		if (sendto(robot_sock_image, buf, buf_len, 0, (struct sockaddr *) &base_image_addr, sizeof(base_image_addr)) < 0) 
		{
			printf("ERROR in sendto");
			exit(1);
		}
		printf("sent %i\n", bytes/1024);
	}

	fclose(f_ptr);

	memset(buf, 0, 1024);
	strcpy(buf, "end");
	buf_len = strlen(buf);
	if (sendto(robot_sock_image, buf, buf_len, 0, (struct sockaddr *) &base_image_addr, sizeof(base_image_addr)) < 0) 
	{
		printf("ERROR in sendto");
		exit(1);
	}
}

void recv_image_at_base()
{
	int num_buf = 256;
	int buf_len = 1024;
	char buf_arr[num_buf][buf_len];
	int buf_loc = 0;

	const char* image_path = "/tmp/test_image.png";

	int robot_image_addr_len = sizeof(robot_image_addr);

	int tot_bytes;
	int packet_size;
	int packets_expected;

	int expect_start = 1;
	int expect_end = 0;

	while (1)
	{
		int failed = 0;
		memset(buf_arr[buf_loc], 0, buf_len);
		int bytes_recv = recvfrom(base_sock_image, buf_arr[buf_loc], buf_len, 0, (struct sockaddr *) &robot_image_addr, &robot_image_addr_len);
		if (bytes_recv <= 0)
		{
			printf("ERROR in recvfrom");
			exit(1);
		}
		printf("recv %i %i\n", buf_loc, bytes_recv);

		if (expect_start)
		{
			int space_loc = -1;
			const char* start_str = "start ";
			for (int i = 0; i < bytes_recv; ++i)
			{
				printf("%i\n", i);
				if (i < strlen(start_str))
				{
					if (buf_arr[buf_loc][i] != start_str[i])
					{
						printf("invalid message received\n%s\n", buf_arr[buf_loc]);
						failed = 1;
						break;
					}
				}
				else
				{
					if (buf_arr[buf_loc][i] == ' ')
					{
						if (space_loc == -1) 
						{
							space_loc = i;
						}
						else
						{
							printf("invalid message received\n%s\n", buf_arr[buf_loc]);
							failed = 1;
							break;
						}
					}
				}
			}


			if (!failed)
			{
				tot_bytes = atoi(buf_arr[buf_loc] + strlen(start_str) - 1);
				packet_size = atoi(buf_arr[buf_loc] + space_loc);
				packets_expected = tot_bytes/packet_size + 1;

				expect_start = 0;
				buf_loc = buf_loc + 1;
			}
		}
		else if (expect_end)
		{
			if (strcmp(buf_arr[buf_loc], "end") != 0)
			{
				printf("not end\n%s\n", buf_arr[buf_loc]);
				expect_end = 0;
			}
			else
			{
				FILE* f_ptr = fopen(image_path, "wb");

				for (int i = 1; i < packets_expected; ++i)
				{
					fwrite(buf_arr[i], sizeof(char), buf_len, f_ptr);
				}

				fwrite(buf_arr[packets_expected], sizeof(char), tot_bytes%packet_size, f_ptr);

				fclose(f_ptr);
			}

			expect_end = 0;
			expect_start = 1;
			buf_loc = 0;
		}
		else
		{
			buf_loc = buf_loc + 1;
			expect_end = (buf_loc > packets_expected);
		}
	}
}

