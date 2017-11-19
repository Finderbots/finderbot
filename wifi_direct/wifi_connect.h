#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// IP addresses for endpoints
#define IP_BASE "192.168.2.1"
#define IP_ROBOT "192.168.2.2"

struct popen2 {
    pid_t child_pid;
    int   from_child, to_child;
};

// allows you to run a program from c interfacing via stdin and stdout 
int popen2(const char *cmdline, struct popen2 *childinfo);

// initiates wifi direct connection on base
void connect_base();

// disconnects the base from all wifi networks
void disconnect_base();

// initiates wifi direct connection on robot
void connect_robot();

// disconnects the robot from all wifi networks
void disconnect_robot();

// searches input string for given mac address. 
int find_mac(const char* out_str, int out_len, const char* mac_str, int mac_len);

#endif /* WIFI_CONNECT_H */