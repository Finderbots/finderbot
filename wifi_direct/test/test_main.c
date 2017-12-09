#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

#include "../wifi_connect.h"
#include "../data_transmission.h"

void base_image()
{
	connect_base();
	printf("base connected\n");

	base_init_image();
	printf("base init success\n");

	recv_image_at_base();
	printf("base done\n");
}

void robot_image()
{
	connect_robot();
	printf("robot connected\n");
	
	robot_init_image();
	printf("robot init success\n");

	sleep(3);
	const char* file_name = "/home/odroid/Downloads/keep-calm-and-love-rohit-12.png";
	for (int i = 0; i < 10; ++i) send_image_from_robot(file_name);
	printf("robot done\n");
}

void base_audio_in()
{
        connect_base();
        printf("base connected\n");

        base_init_audio_in();
        printf("base init success\n");

       // base_recv_audio();
        printf("base done\n");
}

void base_audio_out()
{
	connect_base();
	printf("base connected\n");

	base_init_audio_out();
	printf("base init success\n");

	base_send_audio();
	printf("base done\n");
}

void robot_audio_in()
{
	connect_robot();
	printf("robot connected\n");
	
	robot_init_audio_in();
	printf("robot init success\n");

	robot_recv_audio();
	printf("robot done\n");
}

void robot_audio_out()
{
        connect_robot();
        printf("robot connected\n");

        robot_init_audio_out();
        printf("robot init success\n");

        //robot_send_audio();
        printf("robot done\n");
}


int main()
{
	// call func for device running on
//	robot_audio_in();
connect_robot();

    return 0;
}

