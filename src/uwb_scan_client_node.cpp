#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/UWBScan.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "UWB_Scan_Client");
	ros::NodeHandle nh;

	ros::ServiceClient uwb_scan_client = nh.serviceClient<finderbot::UWBScan>("UWB_Scan");
	finderbot::UWBScan uwb_scan_srv;

	if (uwb_scan_client.call(uwb_scan_srv)) {
		ROS_INFO("Called UWB Scan Service");
	}
	else {
		ROS_ERROR("Failed to call UWB Scan Service");
		return 1;
	}
	ros::spin();
	return 0;
}