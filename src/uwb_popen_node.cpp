#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <fstream>


#define MAX_UWB_LEN 10

const char uwb_program_name[] = "./path/to/exec";
char uwb_message[MAX_UWB_LEN];
std::fstream file;

double * ros_call_uwb_program()
{
	FILE* uwb_out;

	uwb_out = popen(uwb_program_name, "r");
	if (uwb_out == NULL)
	{
		// fuck
	}

	delay(2000); // delay two seconds to give time for the uwb rogram to produce output
	             // delay time should be adjusted according to Sheng's input
	             // might be worth considering more efficent of more ROS was of delaying

	double vital_found = 0; // this is boolean but I want to return an array with
	if (fgets(uwb_message, MAX_UWB_LEN, uwb_out) != NULL)
	{
		vital_found = parse_vital_found(uwb_message);
	}

	double distance = 0;
	if (vital_found == 1)
	{
		if (fgets(uwb_message, MAX_UWB_LEN, uwb_out) != NULL)
		{
			distance = parse_distance(uwb_message);
		}
	}

	bool only_end_line = false;
	if (distance > 0)
	{
		if (fgets(uwb_message, MAX_UWB_LEN, uwb_out) != NULL)
		{
			only_end_line = ((uwb_message[0] == '\n') && (uwb_message[1] == '\0'));
		}
	}

	pclose(uwb_out);

	if (only_end_line)
	{
		// publish vital and distance
		person_and_distance_msg.data.clear();
		person_and_distance_msg.data.push_back(vital_found);
		person_and_distance_msg.data.push_back(distance);
		uwb_publisher.publish(person_and_distance_msg);
		ROS_INFO("Just published [ Person Detected: %zd, Distance: %zd ]",
				 person_and_distance_msg.data[0], person_and_distance_msg.data[1]);
		ros::spinOnce();
	}
	else
	{
		//publish no vital
	}

}

int parse_vital_found(char uwb_message[])
{
	// if first chars as int
	int found = atoi(uwb_message);
	return found;
}

double parse_distance(char uwb_message[])
{
	// if first chars as float
	double dist = atof(uwb_message);
	return dist;
}

