#include <ros/ros.h>
#include <ros/console.h>
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

finderbot::UWB_data uwb_data;
ros::Publisher uwb_publisher;

std::string getLastLine(std::fstream &file) {
	bool keep_looping = true;
	file.seekg(-1,std::ios_base::end);
	while (!keep_looping) {
		char ch;
		file.get(ch);
		if ((int)file.tellg() <= 1) {
			file.seekg(0);
			keep_looping = false;
		}
		else if (ch == '\n') {
			keep_looping = false;
		}
		else {
			file.seekg(-2,std::ios_base::cur);
		}
	}
	std::string last_line;
	getline(file, last_line);
	return last_line;
}

int parseVitals(std::string last_line) {
	std::string delimiter = "V:";
	size_t idx = last_line.find(delimiter) + delimiter.length();
	char c = last_line.at(idx);
	return c-'0';
}

float parseDistance(std::string last_line) {
	std::string delimiter = "D:";
	return std::stof(last_line.substr(last_line.find(delimiter) + delimiter.length(), std::string::npos));
}

void scanUWB(float servo_angle) {
	// Copied from main() function in uwb_main.cpp
	// 6 second wait when you try to get uwb data
	std::cout << "Waiting 12 seconds...\n";
	sleep(1);
	// Read from last line of file

	std::fstream file;
	file.open("/home/parallels/catkin_ws/src/finderbot/src/test.txt"); // Change this to correct filename
	if (file.fail()) std::cout << "FILE FAILED\n";
	else std::cout << "FILE SUCCEEDED\n";
	std::string last_line = getLastLine(file);
	std::cerr << "LAST LINE: " << last_line << "\n";
	file.close();
	
	int uwb_vital = parseVitals(last_line);
	float uwb_distance = parseDistance(last_line);

	// publish 1 (vital detected) or 0 (not)
	//  and distance to vitals (<=0 if not found)
	uwb_data.vital = uwb_vital;
	uwb_data.distance = uwb_distance;
	uwb_data.angle = servo_angle;
	uwb_publisher.publish(uwb_data);
	ROS_INFO("Just published [ Person Detected: %i, Distance: %f ]",
			 uwb_data.vital, uwb_data.distance);
	ros::spinOnce();

	return;
}

// V:i D:f \n

bool rotateAndScan() {

	// Servo init stuff copied from servo.c
	// servo_setup();

	// response.person_found = 0; // worry about this later

	// 1. Rotate the servo to 0 degrees (absolute-90)
	// 		** make sure it blocks the scan function
	// servo_turn0();
	// 2. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB(0);
	// 3. Rotate the servo to 90 degrees (absolute)
	// 		** make sure it blocks the scan function (series not parallel)
	// servo_turn90();
	// 4. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB(90);
	// 5. Rotate the servo to 180 degrees (absolute+90)
	// 		** make sure it blocks the scan function (series not parallel)
	// servo_turn180();
	// 6. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB(180);

	// Middle again
	// servo_turn90();
	scanUWB(90);

	// Left again
	// servo_turn0();
	scanUWB(0);

	return true;
}

int main(int argc, char** argv)
{
	// 0. Init
	// 		Get absolute current row, col, theta
    ros::init(argc, argv, "UWB_Scan");
	ros::NodeHandle nh;
	ros::Publisher uwb_publisher = nh.advertise<finderbot::UWB_data>("/UWB_data", 1, true);

	// ros::ServiceServer service = nh.advertiseService("UWB_Scan", rotateAndScan);
    std::cout << "Ready to scan UWB for people." << std::endl;
	// while (1){
		rotateAndScan();
	// }
	// ros::spin();

	return 0;
}
