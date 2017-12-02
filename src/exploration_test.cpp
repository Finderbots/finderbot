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

    std::vector<double> map(global_width*global_width, -1);
    map[map_utils::getOffsetRowCol(0, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(2, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(0, 3, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 3, global_width)] = 90;
    map[map_utils::getOffsetRowCol(2, 3, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 3, global_width)] = 90;


    map[map_utils::getOffsetRowCol(0, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(1, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(2, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(3, 2, global_width)] = 0;


    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     1,
                     0);
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "global_width = " << global_width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), global_width) << ", "
                        << map_utils::colFromOffset(path.front(), global_width) <<")" << std::endl;
}

void test_harder_hallway()
{
    std::cout << "HARDER HALLWAY TEST" << std::endl;

    size_t source_x = 0;
    size_t source_y = 3;

    std::vector<double> map(global_width*global_width, -1);
    //left wall
    map[map_utils::getOffsetRowCol(0, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(2, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 1, global_width)] = 90;

    //right wall
    map[map_utils::getOffsetRowCol(0, 4, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 4, global_width)] = 90;
    map[map_utils::getOffsetRowCol(2, 4, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 4, global_width)] = 90;

    //halls
    map[map_utils::getOffsetRowCol(0, 2, global_width)] = 0; 
    map[map_utils::getOffsetRowCol(1, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(2, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(3, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(4, 2, global_width)] = 0;


    map[map_utils::getOffsetRowCol(0, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(1, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(2, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(3, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(4, 3, global_width)] = 0;

    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     1,
                     0);
    
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "global_width = " << global_width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), global_width) << ", "
                        << map_utils::colFromOffset(path.front(), global_width) <<")" << std::endl;

}

void test_junction()
{
    std::cout << "TEST JUNCTION" <<std::endl;

    size_t source_x = 0;
    size_t source_y = 3;

    std::vector<double> map(global_width*global_width, -1);
    //left wall
    map[map_utils::getOffsetRowCol(0, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 1, global_width)] = 90;
    
    //right wall
    map[map_utils::getOffsetRowCol(0, 4, global_width)] = 90;
    map[map_utils::getOffsetRowCol(1, 4, global_width)] = 90;

    //top wall
    map[map_utils::getOffsetRowCol(3, 0, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 1, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 2, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 3, global_width)] = 90;
    map[map_utils::getOffsetRowCol(3, 4, global_width)] = 90;

    map[map_utils::getOffsetRowCol(0, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(0, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(1, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(1, 3, global_width)] = 0;
    map[map_utils::getOffsetRowCol(2, 2, global_width)] = 0;
    map[map_utils::getOffsetRowCol(2, 3, global_width)] = 0;


    Planner planner(map);
    planner.setPose(source_x, source_y);

    std::vector<frontier_t> frontiers;
    findMapFrontiers(planner, 
                     frontiers,
                     1,
                     0);
    
    std::cout << "FOUND " << frontiers.size() << " FRONTIERS" << std::endl;
    std::vector <size_t> path = exploreFrontiers(planner, frontiers);

    std::cout << "global_width = " << global_width << std::endl;

    std::cout << "GOAL at (" << map_utils::rowFromOffset(path.front(), global_width) << ", "
                        << map_utils::colFromOffset(path.front(), global_width) <<")" << std::endl;


      
}


int main(int argc, char** argv)
{
    test_simple_hallway();
    test_harder_hallway();
    test_junction();
    return 0;
}