#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <finderbot/DistanceGrid.h>
#include <finderbot/map_utils.h>

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_grid_test");
    ros::NodeHandle nh;

    size_t global_height = 6;
    size_t global_width = 6;

    distance_grid::DistanceGrid grid(global_width, global_height);

    nav_msgs::OccupancyGrid map;
    map.data.resize(9);
    map.data.assign(global_height* global_width,0);

    map.data[map_utils::getOffsetRowCol(2,2,global_width)] = 100;

    std::vector<double> vec;
    for (size_t i = 0; i < map.data.size(); i++)
    {
        vec.push_back(map.data[i]);
    }
    grid.setDistances(vec);


    for (int i = global_height-1 ; i >= 0; i--)
    {
        for (size_t j = 0; j < global_width; j++)
        {
            size_t idx = map_utils::getOffsetRowCol(i, j, global_width);
            std::cout << grid[idx] <<" ";
        }

        std::cout << std::endl;
    }

}