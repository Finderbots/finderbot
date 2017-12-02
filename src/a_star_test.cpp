#include <finderbot/planner.h>
#include <finderbot/map_utils.h>
#include <iostream>
#include <vector>


void printPath(std::vector<int> & path) {
	int row = 0;
	int col = 0;
	std::cout << "PRINTING PATH:\n";
	while (!path.empty()) {
		row = map_utils::rowFromOffset(path.back(), global_width);
		col = map_utils::colFromOffset(path.back(), global_width);
		path.pop_back();
		std::cout << "(" << row << "," << col << ")\n";
	}
	return;
}

// Test 1:
//	IN:
// 	 source == goal == (0,0),
// 	 no obstacles
// 	OUT:
//   [ 0 , 0 ]
void testSanity() {
	std::cout << "STARTING SANITY TEST\n";
	std::vector<double> occupancy_grid(global_width * global_width, 0);
	int source_x = 0;
	int source_y = 0;
	int goal_x = 0;
	int goal_y = 0;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	printPath(*planner->aStar(goal_x, goal_y));
	return;
}

// Test 2:
//	IN:
// 	 source == (0,0),
//   goal == (10,10),
//   no obstacles
// 	OUT:
//   [ 10 , 10 , 9 , 9 , ... , 0 , 0 ]
void testDiagonal() {
	std::cout << "STARTING DIAGONAL PATH TEST\n";
	std::vector<double> occupancy_grid(global_width * global_width, 0);
	int source_x = 0;
	int source_y = 0;
	int goal_x = 10;
	int goal_y = 10;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	printPath(*planner->aStar(goal_x, goal_y));
	return;
}

// Test 3:
//  IN:
//   source == (0,3),
//   goal == (10,3),
//   wall from (3,6) to (3,0)
// 	OUT:
//   [ 10 , 3 , ... , 0 , 3 ]
void testVerticalWall() {
	std::cout << "STARTING VERTICAL WALL TEST\n";
	std::vector<double> occupancy_grid(global_width * global_width, 0);
	occupancy_grid[map_utils::getOffsetRowCol(3, 0, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 1, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 2, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 3, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 4, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 5, global_width)] = 90;
	int source_x = 0;
	int source_y = 3;
	int goal_x = 10;
	int goal_y = 3;
	Planner * planner = new Planner(occupancy_grid);
	planner->setPose(source_x, source_y);
	printPath(*planner->aStar(goal_x, goal_y));
	return;
}

void testInvalidPath() {
	std::cout << "STARTING SANITY TEST\n";
	std::vector<double> occupancy_grid(global_width * global_width, 0);
	occupancy_grid[map_utils::getOffsetRowCol(3, 0, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 1, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 2, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(3, 3, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(2, 3, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(1, 3, global_width)] = 90;
	occupancy_grid[map_utils::getOffsetRowCol(0, 3, global_width)] = 90;
	int source_x = 0;
	int source_y = 0;
	int goal_x = 5;
	int goal_y = 5;
	Planner * planner = new Planner(occupancy_grid);
	printPath(*planner->aStar(goal_x, goal_y, source_x, source_y));
	return;	
}

// NO ROS NECESSARY!!!
int main(int argc, char** argv) {
	testSanity();
	testDiagonal();
	testVerticalWall();
	testInvalidPath();
	return 0;
}
