#include <finderbot/path_execution.h>
#include <finderbot/planner.h>
#include <finderbot/map_utils.h>
#include <iostream>
#include <vector>



void printPath(std::vector<size_t> * path) {
	if (path == NULL) {
		std::cout << "No Path to Print\n";
		return;
	}
	size_t row = 0;
	size_t col = 0;
	std::cout << "PRINTING PATH:\n";
	while (!path->empty()) {
		row = map_utils::rowFromOffset(path->back(), global_width);
		col = map_utils::colFromOffset(path->back(), global_width);
		path->pop_back();
		std::cout << "(" << row << "," << col << ")\n";
	}
	return;
}

void testStraightHorizontal() {
	std::cout << "STARTING HORIZONTAL PATH TEST\n";
    ros::NodeHandle nh;
    // command_velocities_pub = nh.advertise<geometry_msgs::Twist>("command_velocities", 1, true);
    ros::Rate loop_rate(10);
	nav_msgs::OccupancyGrid occupancy_grid;
	occupancy_grid.info.width = 1000;
	occupancy_grid.info.height = 1000;
	occupancy_grid.data.assign(occupancy_grid.info.width*occupancy_grid.info.height, 0);

	size_t source_x = 0;
	size_t source_y = 0;
	size_t goal_x = 0;
	size_t goal_y = 900;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	// printPath(planner->aStar(goal_x, goal_y));
	Executor path_executor("world", "laser_frame");


	ros::Subscriber global_map_handler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, &Executor::handleGlobalMap, &path_executor);

	ROS_INFO("Execution Node Starting");

	while (ros::ok())
	{
		if (!path_executor.initialized()) {
			ROS_INFO("Map Uninitialized: Continue");
			ros::spinOnce();
			continue;
		}
		std::vector<size_t> path = planner->aStar(goal_x, goal_y);

		ROS_INFO("EXECUTE PATH OF LENGTH %zd", path.size());
		path_executor.pathExecution(path);
	}
	return;
}

void testStraightDiagonal() {
	std::cout << "STARTING DIAGONAL PATH TEST\n";
    ros::NodeHandle nh2;
    // command_velocities_pub = nh2.advertise<geometry_msgs::Twist>("command_velocities", 1, true);
    ros::Rate loop_rate(10);
	nav_msgs::OccupancyGrid occupancy_grid;
	occupancy_grid.info.width = 1000;
	occupancy_grid.info.height = 1000;
	occupancy_grid.data.assign(occupancy_grid.info.width*occupancy_grid.info.height, 0);

	size_t source_x = 0;
	size_t source_y = 0;
	size_t goal_x = 900;
	size_t goal_y = 900;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	// printPath(planner->aStar(goal_x, goal_y));
	Executor path_executor("world", "laser_frame");


	ros::Subscriber global_map_handler = nh2.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, &Executor::handleGlobalMap, &path_executor);

	ROS_INFO("Execution Node Starting");

	while (ros::ok())
	{
		if (!path_executor.initialized()) {
			ROS_INFO("Map Uninitialized: Continue");
			ros::spinOnce();
			continue;
		}
		std::vector<size_t> path = planner->aStar(goal_x, goal_y);

		ROS_INFO("EXECUTE PATH OF LENGTH %zd", path.size());
		path_executor.pathExecution(path);
	}
	return;
}

void buildVerticalWall(size_t row, size_t col_start, size_t col_end, nav_msgs::OccupancyGrid & occupancy_grid) {
	for (int i = col_start; i < col_end - col_start; ++i)
	{
		occupancy_grid.data[map_utils::getOffsetRowCol(row, i, global_width)] = 90;
	}
	return;
}

void testPathWithTurns() {
	std::cout << "STARTING PATH WITH TURNS TEST\n";
	ros::NodeHandle nh3;
	// command_velocities_pub = nh3.advertise<geometry_msgs::Twist>("command_velocities", 1, true);
	ros::Rate loop_rate(10);
	nav_msgs::OccupancyGrid occupancy_grid;
	occupancy_grid.info.width = 1000;
	occupancy_grid.info.height = 1000;
	occupancy_grid.data.assign(occupancy_grid.info.width*occupancy_grid.info.height, 0);

	buildVerticalWall(250, 50, 499, occupancy_grid);
	size_t source_x = 0;
	size_t source_y = 0;
	size_t goal_x = 500;
	size_t goal_y = 500;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	// printPath(planner->aStar(goal_x, goal_y));
	Executor path_executor("world", "laser_frame");


	ros::Subscriber global_map_handler = nh3.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, &Executor::handleGlobalMap, &path_executor);

	ROS_INFO("Execution Node Starting");

	while (ros::ok())
	{
		if (!path_executor.initialized()) {
			ROS_INFO("Map Uninitialized: Continue");
			ros::spinOnce();
			continue;
		}
		std::vector<size_t> path = planner->aStar(goal_x, goal_y);

		ROS_INFO("EXECUTE PATH OF LENGTH %zd", path.size());
		path_executor.pathExecution(path);
	}
	return;
}

int main(int argc, char const *argv[])
{
	ros::init(argc, argv, "test_path_exec_node");

	// Uncomment 1 at a time?
	testStraightHorizontal();
	// testStraightDiagonal();
	// testPathWithTurns();
	return 0;
}