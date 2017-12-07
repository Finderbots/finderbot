#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/uwb.h>
#include <finderbot/UWB/ModuleConnector.hpp>
// #include <finderbot/path_execution_node.h>
#include <ctime>
#include <iostream>
#include <unistd.h>
#include <finderbot/servo.h>

XeThru::Uwb * uwb_;
ros::Publisher uwb_publisher;

void scanUWB(XeThru::Uwb * uwb_) {
	// Copied from main() function in uwb_main.cpp
	// Does it need to call uwb_get_data thrice?
	uwb_->uwb_get_data();
	sleep(5);
	// publish 1 (vital detected) or 0 (not)
	// uwb_publisher.publish(uwb_->state);
	// ros::spinOnce();
	return;
}

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
	scanUWB(uwb_);
	if (uwb_->state == 1) {
		ROS_INFO("Vital signal found on the left!");
	}
	// 3. Rotate the servo to 90 degrees (absolute)
	// 		** make sure it blocks the scan function (series not parallel)
	servo_turn90();
	// 4. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB(uwb_);
	if (uwb_->state == 1) {
		ROS_INFO("Vital signal found straight ahead!");
	}
	// 5. Rotate the servo to 180 degrees (absolute+90)
	// 		** make sure it blocks the scan function (series not parallel)
	servo_turn0();
	// 6. Once it finishes rotating, scan new uwb data
	// 		use functions as in uwb_main.cpp
	// 		uwb_->state says if there is a person
	// 		block task 3 until you have finished scanning
	scanUWB(uwb_);
	if (uwb_->state == 1) {
		ROS_INFO("Vital signal found on the right!");
	}


	return true;
}

int main(int argc, char** argv)
{
	// 0. Init
	// 		Get absolute current row, col, theta
	ros::init(argc, argv, "UWB_Scan");
	ros::NodeHandle nh;
    // UWB init stuff copied from main() function in uwb_main.cpp
    const std::string device_name = "/dev/ttySAC0";
    const unsigned int log_level = 0;

    XeThru::ModuleConnector mc(device_name, log_level);
    //X4M300 &x4m300 = mc.get_x4m300();
    uwb_ = new XeThru::Uwb(mc);
    uwb_->uwb_setup();

	// ros::ServiceServer service = nh.advertiseService("UWB_Scan", rotateAndScan);
	ROS_INFO("Ready to scan UWB for people.");
	while (ros::ok()){
		rotateAndScan();
	}
	// ros::spin();

	return 0;
}