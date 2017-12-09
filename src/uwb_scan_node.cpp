#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/uwb.h>
#include <finderbot/UWB/ModuleConnector.hpp>
// #include <finderbot/path_execution_node.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>     /* atoi */
#include <unistd.h>
#include <finderbot/UWB_data.h>
#include <finderbot/servo.h>

ros::Publisher uwb_publisher;
ros::NodeHandle nh;
finderbot::UWB_data uwb_data;

string getLastLine(fstream file) {
	bool keep_looping = true;
	file.seekg();
	while (!keep_looping) {
		char ch;
		file.get(ch);
		if ((int)file.tellg() <= 1) {
			file.seekg(0);
			keep_looping = false;
		}
		else if (ch == "\n") {
			keep_looping = false;
		}
		else {
			file.seekg(-2,ios_base::cur)
		}
	}
	string last_line;
	getline(file, last_line);
	return last_line;
}

void scanUWB(XeThru::Uwb * uwb_) {
	// Copied from main() function in uwb_main.cpp
	// 6 second wait when you try to get uwb data
	std::cout << "Waiting 12 seconds...\n";
	sleep(1000);
	// Read from last line of file
	file.open("test.txt"); // Change this to correct filename
	string last_line = getLastLine(file);
	file.close();
	
	int uwb_vital = parseVitals(last_line);
	float uwb_distance = parseDistance(last_line);

	// publish 1 (vital detected) or 0 (not)
	//  and distance to vitals (<=0 if not found)
	uwb_data.vital = uwb_vital;
	uwb_data.distance = uwb_distance;
	uwb_publisher.publish(uwb_data);
	ROS_INFO("Just published [ Person Detected: %zd, Distance: %zd ]",
			 uwb_data.vital, uwb_data.distance);
	ros::spinOnce();

	return;
}

int parseVitals(string last_line) {
	string delimiter = "V:";
	return std::atoi(last_line.at(last_line.find(delimiter) + delimiter.length()));
}

float parseDistance(string last_line) {
	string delimiter = "D:";
	return std::stof(last_line.substr(last_line.find(delimiter) + delimiter.length(), std::string::npos));
}

// V:i D:f \n

bool rotateAndScan() {

	// Servo init stuff copied from servo.c
	servo_setup();

	// response.person_found = 0; // worry about this later

	// 1. Rotate the servo to 0 degrees (absolute-90)
	// 		** make sure it blocks the scan function
	servo_turn0();
	// 2. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB();
	// 3. Rotate the servo to 90 degrees (absolute)
	// 		** make sure it blocks the scan function (series not parallel)
	servo_turn90();
	// 4. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB();
	// 5. Rotate the servo to 180 degrees (absolute+90)
	// 		** make sure it blocks the scan function (series not parallel)
	servo_turn180();
	// 6. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB();

	// Middle again
	servo_turn90();
	scanUWB();

	// Left again
	servo_turn0();
	scanUWB();

	return true;
}

int main(int argc, char** argv)
{
	// 0. Init
	// 		Get absolute current row, col, theta
    ros::init(argc, argv, "UWB_Scan");

	// ros::ServiceServer service = nh.advertiseService("UWB_Scan", rotateAndScan);
    std::cout << "Ready to scan UWB for people." << std::endl;
	// while (1){
		rotateAndScan();
	// }
	// ros::spin();

	return 0;
}
