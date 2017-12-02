#include <path_execution_node.h>

void path_execution() {
	int current_row = 0;
	int current_col = 0;
	int goal_row = 0;
	int goal_col = 0;
	while (!path_coordinates_.empty()) {
		// Pop from the end so you get column first then row
		goal_row = map_utils::rowFromOffset(path_coordinates_.back());
		goal_col = map_utils::colFromOffset(path_coordinates_.back());
		path_coordinates_.pop_back();
		while ((current_row != goal_row) && (current_col != goal_col)) {
			// UPDATE Current position everytime
			// 	current_row and current_col will come from readings from the SLAM algorithm
			// 	convert the values from slam using the reverse ray caster function below

			// We rotate to point straight at the next point in the path then go straight to it
			// If there is error in theta from current position to goal
			// correct that
			// Else if there is error in x,y from current position to goal
			// correct that
		}
		path_coordinates_.
	}
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

// Make a function that does the reverse of the ray caster code's functionality
//   ray caster takes a (row,col) coordinate and outputs the meters
//   make something that does the opposite, takes in (x,y) meters and output (row,col)
