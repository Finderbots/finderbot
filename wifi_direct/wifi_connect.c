#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "wifi_connect.h"

const char* base_mac = "0c:8b:fd:75:7b:dd";
const char* robot_mac = "00:c0:ca:96:cb:94";

const char* kill_wpa_supplicant = "sudo killall wpa_supplicant";
const char* start_wpa_supplicant = "sudo wpa_supplicant -i %s -D %s -c %s -Bd";
const char* start_wpa_cli = "sudo wpa_cli -i %s";
const char* start_network = "sudo ifconfig p2p-%s-0 %s up";

const char* base_interface = "wlp2s0";
const char* base_driver = "nl80211";
const char* base_conf = "/etc/eecs473/wpa_supplicant.conf";

const char* robot_interface = "wlan0";
const char* robot_driver = "nl80211";
const char* robot_conf = "/etc/eecs473/wpa_supplicant.conf";

int buf_len = 4096;
char read_buf[4096];

const char* p2p_find = "p2p_find\n";
const char* p2p_peers = "p2p_peers\n";
const char* p2p_connect_base = "p2p_connect %s pbc auth go_intent=1 freq=%i\n";
const char* p2p_connect_robot = "p2p_connect %s pbc go_intent=1 freq=%i\n";
const char* p2p_quit = "quit";
int p2p_freq = 2412; // channel 
const char* response_end = "OK\n> ";

void strip_kernel_output(char* cli_out)
{
    int in_loc = 0;
    int out_loc = 0;
    int kernel_out = 0;

    while (cli_out[in_loc] != '\0')
    {
        if (kernel_out == 1)
        {
            if (cli_out[in_loc] == '\n')
            {
                kernel_out = 0;
            }
        }
        if (kernel_out == 0)
        {
            if (cli_out[in_loc] == '<')
            {
                kernel_out = 1;
                out_loc = out_loc - 4;
            }
            else
            { 
                cli_out[out_loc] = cli_out[in_loc];
                out_loc = out_loc + 1;
            }
        }

        in_loc = in_loc + 1;
    }
    cli_out[out_loc] = '\0';
}

int popen2(const char *cmdline, struct popen2 *childinfo) {
    pid_t p;
    int pipe_stdin[2], pipe_stdout[2];

    if(pipe(pipe_stdin)) return -1;
    if(pipe(pipe_stdout)) return -1;

    printf("pipe_stdin[0] = %d, pipe_stdin[1] = %d\n", pipe_stdin[0], pipe_stdin[1]);
    printf("pipe_stdout[0] = %d, pipe_stdout[1] = %d\n", pipe_stdout[0], pipe_stdout[1]);

    p = fork();
    if(p < 0) return p; /* Fork failed */
    if(p == 0) { /* child */
        close(pipe_stdin[1]);
        dup2(pipe_stdin[0], 0);
        close(pipe_stdout[0]);
        dup2(pipe_stdout[1], 1);
        execl("/bin/sh", "sh", "-c", cmdline, NULL);
        perror("execl"); 
        exit(99);
    }
    childinfo->child_pid = p;
    childinfo->to_child = pipe_stdin[1];
    childinfo->from_child = pipe_stdout[0];
    return 0; 
}

void connect_base()
{
	char exec_str[256];

	sprintf(exec_str, start_wpa_supplicant, base_interface, base_driver, base_conf);
	system(exec_str);
	sleep(1);
	printf("started wpa_supplicant\n");

	strcpy(exec_str, kill_wpa_supplicant);
	system(exec_str);
	sleep(1);
	printf("killed wpa_supplicant\n");

	sprintf(exec_str, start_wpa_supplicant, base_interface, base_driver, base_conf);
	system(exec_str);
	sleep(1);
	printf("started wpa_supplicant\n");

	sprintf(exec_str, start_wpa_cli, base_interface);
	struct popen2 wpa_cli;
	popen2(exec_str, &wpa_cli);
	sleep(1);
	printf("started wpa_cli\n");
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
	sleep(1);

	strcpy(exec_str, p2p_find);
	write(wpa_cli.to_child, exec_str, strlen(p2p_find));
	sleep(1);
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
        strip_kernel_output(read_buf);
	sleep(1);
	strcat(exec_str, response_end); // set exec_str to expected output
	if (strcmp(read_buf, exec_str))
	{
		printf("p2p_find response wasn't OK\n");
		printf("%s\n", read_buf);
		exit(1);
	}
	printf("ran p2p_find\n");

	strcpy(exec_str, p2p_peers);
	int found_robot_mac = 0;
	while (!found_robot_mac)
	{
		write(wpa_cli.to_child, exec_str, strlen(p2p_peers));
		sleep(1);
		memset(read_buf, 0, buf_len);
		read(wpa_cli.from_child, read_buf, buf_len);
                strip_kernel_output(read_buf);
		sleep(1);
		found_robot_mac = find_mac(read_buf, buf_len, robot_mac, strlen(robot_mac));
	}
	printf("ran p2p_peers\n");

	sprintf(exec_str, p2p_connect_base, robot_mac, p2p_freq);
	write(wpa_cli.to_child, exec_str, strlen(exec_str));
	sleep(1);
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
        strip_kernel_output(read_buf);
	sleep(1);
	strcat(exec_str, response_end); // set exec_str to expected output
	if (strcmp(read_buf, exec_str))
	{
		printf("p2p_connect response wasn't OK\n");
		printf("%s\n", read_buf);
		exit(1);
	}
	printf("ran p2p_connect\n");

	strcpy(exec_str, p2p_quit);
	write(wpa_cli.to_child, exec_str, strlen(p2p_quit));
	sleep(1);
	printf("quit wpa_cli\n");

	sprintf(exec_str, start_network, base_interface, IP_BASE);
	system(exec_str);
	sleep(1);
	printf("p2p up on IP_BASE %s\n", IP_BASE);
}

void disconnect_base()
{
	char exec_str[256];
	strcpy(exec_str, kill_wpa_supplicant);
	system(exec_str);
	sleep(1);
	printf("killed wpa_supplicant\n");
}

void connect_robot()
{
	char exec_str[256];

	sprintf(exec_str, start_wpa_supplicant, robot_interface, robot_driver, robot_conf);
	system(exec_str);
	sleep(1);
	printf("started wpa_supplicant\n");

	strcpy(exec_str, kill_wpa_supplicant);
	system(exec_str);
	sleep(1);
	printf("killed wpa_supplicant\n");

	sprintf(exec_str, start_wpa_supplicant, robot_interface, robot_driver, robot_conf);
	system(exec_str);
	sleep(1);
	printf("started wpa_supplicant\n");

	sprintf(exec_str, start_wpa_cli, robot_interface);
	struct popen2 wpa_cli;
	popen2(exec_str, &wpa_cli);
	sleep(1);
	printf("started wpa_cli\n");
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
	sleep(1);

	strcpy(exec_str, p2p_find);
	write(wpa_cli.to_child, exec_str, strlen(p2p_find));
	sleep(1);
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
        strip_kernel_output(read_buf);
	sleep(1);
	strcat(exec_str, response_end); // set exec_str to expected output
	if (strcmp(read_buf, exec_str))
	{
		printf("p2p_find response wasn't OK\n");
		printf("%s\n", read_buf);
		exit(1);
	}
	printf("ran p2p_find\n");

	strcpy(exec_str, p2p_peers);
	int found_base_mac = 0;
	while (!found_base_mac)
	{
		write(wpa_cli.to_child, exec_str, strlen(p2p_peers));
		sleep(1);
		memset(read_buf, 0, buf_len);
		read(wpa_cli.from_child, read_buf, buf_len);
                strip_kernel_output(read_buf);
		sleep(1);
		found_base_mac = find_mac(read_buf, buf_len, base_mac, strlen(base_mac));
	}
	printf("ran p2p_peers\n");

	sleep(5);

	sprintf(exec_str, p2p_connect_robot, base_mac, p2p_freq);
	write(wpa_cli.to_child, exec_str, strlen(exec_str));
	sleep(1);
	memset(read_buf, 0, buf_len);
	read(wpa_cli.from_child, read_buf, buf_len);
        strip_kernel_output(read_buf);
	sleep(1);
	strcat(exec_str, response_end); // set exec_str to expected output
	if (strcmp(read_buf, exec_str))
	{
		printf("p2p_connect response wasn't OK\n");
		printf("%s\n", read_buf);
		exit(1);
	}
	printf("ran p2p_connect\n");

	strcpy(exec_str, p2p_quit);
	write(wpa_cli.to_child, exec_str, strlen(p2p_quit));
	sleep(1);
	printf("quit wpa_cli\n");

	sprintf(exec_str, start_network, robot_interface, IP_ROBOT);
	system(exec_str);
	sleep(1);
	printf("p2p up on IP_ROBOT %s\n", IP_ROBOT);
}

void disconnect_robot()
{
	char exec_str[256];
	strcpy(exec_str, kill_wpa_supplicant);
	system(exec_str);
	sleep(1);
	printf("killed wpa_supplicant\n");
}

int find_mac(const char* out_str, int out_len, const char* mac_str, int mac_len)
{
	int mac_index = -1;
	for (int i = 0; i < out_len; ++i)
	{
		if (mac_index >= 0)
		{
			if (out_str[i] == mac_str[mac_index])
			{
				mac_index = mac_index + 1;
				if (mac_index == mac_len)
				{
					// macs match - return 1 for true
					return 1;
				}
			}
			else
			{
				mac_index = -1;
			}
		}

		if (out_str[i] == '\n')
		{
			mac_index = 0;
		}

		if (out_str[i] == '>')
		{
			return 0;
		}
	}

	return 0;
}


