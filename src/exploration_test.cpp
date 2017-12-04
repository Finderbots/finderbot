#include <finderbot/planner.h>
#include <finderbot/exploration.h>
#include <finderbot/map_utils.h>
#include <iostream>
#include <vector>


void test_simple_hallway()
{
    std::cout << "SIMPLE HALLWAY TEST" << std::endl;

    size_t source_x = 0;
    size_t source_y = 2;

    nav_msgs::OccupancyGrid map;
    map.info.width = 1000;
    map.info.height = 1000;
    map.data.assign(map.info.width * map.info.height, -1);
    map.info.resolution = 1;

    map.data[map_utils::getOffsetRowCol(0, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(2, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(0, 3, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 3, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(2, 3, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 3, map.info.width)] = 90;


    map.data[map_utils::getOffsetRowCol(0, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(1, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(2, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(3, 2, map.info.width)] = 0;


    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     0);
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "map.info.width = " << map.info.width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), map.info.width) << ", "
                        << map_utils::colFromOffset(path.front(), map.info.width) <<")" << std::endl;
}

void test_harder_hallway()
{
    std::cout << "HARDER HALLWAY TEST" << std::endl;

    size_t source_x = 0;
    size_t source_y = 3;

    // std::vector<double> map(map.info.width*map.info.width, -1);

    nav_msgs::OccupancyGrid map;
    map.info.width = 1000;
    map.info.height = 1000;
    map.data.assign(map.info.width * map.info.height, -1);
    map.info.resolution = 1;

    //left wall
    map.data[map_utils::getOffsetRowCol(0, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(2, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 1, map.info.width)] = 90;

    //right wall
    map.data[map_utils::getOffsetRowCol(0, 4, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 4, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(2, 4, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 4, map.info.width)] = 90;

    //halls
    map.data[map_utils::getOffsetRowCol(0, 2, map.info.width)] = 0; 
    map.data[map_utils::getOffsetRowCol(1, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(2, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(3, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(4, 2, map.info.width)] = 0;


    map.data[map_utils::getOffsetRowCol(0, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(1, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(2, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(3, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(4, 3, map.info.width)] = 0;

    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     0);
    
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "map.info.width = " << map.info.width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), map.info.width) << ", "
                        << map_utils::colFromOffset(path.front(), map.info.width) <<")" << std::endl;

}

void test_junction()
{
    std::cout << "TEST JUNCTION" <<std::endl;

    size_t source_x = 0;
    size_t source_y = 3;

    // std::vector<double> map(map.info.width*map.info.width, -1);
    nav_msgs::OccupancyGrid map;
    map.info.width = 1000;
    map.info.height = 1000;
    map.data.assign(map.info.width * map.info.height, -1);
    map.info.resolution = 1;
    
    //left wall
    map.data[map_utils::getOffsetRowCol(0, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 1, map.info.width)] = 90;
    
    //right wall
    map.data[map_utils::getOffsetRowCol(0, 4, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(1, 4, map.info.width)] = 90;

    //top wall
    map.data[map_utils::getOffsetRowCol(3, 0, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 1, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 2, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 3, map.info.width)] = 90;
    map.data[map_utils::getOffsetRowCol(3, 4, map.info.width)] = 90;

    map.data[map_utils::getOffsetRowCol(0, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(0, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(1, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(1, 3, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(2, 2, map.info.width)] = 0;
    map.data[map_utils::getOffsetRowCol(2, 3, map.info.width)] = 0;


    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     0);
    
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "map.info.width = " << map.info.width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), map.info.width) << ", "
                        << map_utils::colFromOffset(path.front(), map.info.width) <<")" << std::endl;


      
}


int main(int argc, char** argv)
{
    test_simple_hallway();
    test_harder_hallway();
    test_junction();
    return 0;
}