#include <path_execution_node.h>

#define PI 3.14159265

// Make a function that does the reverse of the ray caster code's functionality
//   ray caster takes a (row,col) coordinate and outputs the meters
//   make something that does the opposite, takes in (x,y) meters and output (row,col)

// Applies command velocities until you are within a threshold of desired angle
void Executor::turnTheta(int goal_theta) {
	if (goal_theta > current_theta_) {
		cmd_.angular.z = -0.05; // LEFT
	}
	else if (goal_theta < current_theta_) {
		cmd_.angular.z = 0.05; // RIGHT
	}
	while (!thetaCloseEnough(goal_theta)) {
		command_velocities_pub.publish(cmd_);
	}
	return;
}

// Returns a command velocity to move forward distance in feet (negative for backwards)
void Executor::drive(int goal_row, int goal_col) {
	cmd_.linear.x = -0.03;
	while (!coordinatesCloseEnough)
		command_velocities_pub.publish(cmd_);
	return;
}

// Input is a radius to the goal that you consider close enough
bool Executor::closeEnoughToGoal(int radius, int goal_row, int goal_col) {
	return distance(goal_row, goal_col, current_row_, current_col_) < radius;
}

void Executor::goToNextNodeInPath(int goal_row, int goal_col) {
	// current_row_ = /* GET FROM SLAM POSE */;
	// current_col_ = /* GET FROM SLAM POSE */;
	// current_theta_ = /* GET FROM SLAM POSE */;
	double goal_theta = atan((goal_row-current_row_)/(goal_col-current_col_)) * (180 / PI); // RAD->DEGREE CONVERSION
	
	while (!closeEnoughToGoal(0.5, goal_row, goal_col)) {
		cmd_.linear.x = 0;
		cmd_.angular.z = 0;
		// We rotate to aim straight at the next point in the path then go straight to it
		turnTheta(goal_theta);
		// Do we need to wait till you finish turning? Or does it queue commands?
		// Is there a way to have it queue instead of one at a time?
		drive(goal_row, goal_col);

		// 	convert the current pose values from slam using the reverse ray caster function
		// current_row_ = /* GET FROM SLAM POSE */;
		// current_col_ = /* GET FROM SLAM POSE */;
		// current_theta_ = /* GET FROM SLAM POSE */;
	}
	return;
}

void Executor::pathExecution(std::vector<int> * path) {

	int goal_row = 0;
	int goal_col = 0;
	while (!path.empty() && ros::ok()) {
		goal_row = map_utils::rowFromOffset(path.back());
		goal_col = map_utils::colFromOffset(path.back());
		path.pop_back();
		goToNextNodeInPath(goal_row, goal_col);
	    ros::spin();
	}
	return;
}

// subscribes to both the slam and the explore nodes
// 	 gets the pose from slam
//   gets the destination from the explore node
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;
	ros::Publisher command_velocities_pub = nh.advertise<geometry_msgs::Twist>("command_velocities", 1, true);
	ros::Rate loop_rate(10);

    std::vector<int> global_map(global_width * global_width, 0);
    // Initialize nodes array to the size of the map
	Planner* planner = new Planner(global_map);
	pathExecution(planner->aStar(goal_x, goal_y, source_x, source_y));

    return 0;
}

