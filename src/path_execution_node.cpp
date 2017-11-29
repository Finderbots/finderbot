#include <path_execution_node.h>

while (!path_coordinates_.empty()) {
	while (current_pose != goal) {

	}
	path_coordinates_.
}

// subscribes to both the slam and the explore nodes
// 	 gets the pose from slam
//   gets the destination from the explore node
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;

    // Just made a 1000 x 1000 map with no obstacles
    std::vector<int> global_map(global_width * global_width, 0);
    // Initialize nodes array to the size of the map
	Planner* planner = new Planner(global_map);

	ros::ServiceServer service = nh.advertiseService("get_path", planner->pathCb);
	ROS_INFO("Ready to print path coordinates.");
    ros::spin();

    return 0;
}