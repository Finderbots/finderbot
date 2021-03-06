#ifndef DISTANCE_GRID_H_
#define DISTANCE_GRID_H_

#include <vector>
#include <queue>
#include <finderbot/map_utils.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

namespace distance_grid{

class DistanceGrid
{
    static size_t global_width;
    static size_t global_height;

    struct DistNode
    {
        double distance;
        size_t cell_idx;
        size_t obstacle_idx;


        bool operator<(const DistNode& rhs) const
        {
            return distance < rhs.distance;
        }

        DistNode(size_t cell_idx, size_t obstacle_idx)
        : 
        cell_idx(cell_idx),
        obstacle_idx(obstacle_idx)
        {
            if (cell_idx == obstacle_idx) distance = 0;

            else
            {
                size_t x_cell = map_utils::rowFromOffset(cell_idx, global_width);
                size_t y_cell = map_utils::colFromOffset(cell_idx, global_width);
                size_t x_obs = map_utils::rowFromOffset(obstacle_idx, global_width);
                size_t y_obs = map_utils::colFromOffset(obstacle_idx, global_width);

                // int delta_x = (x_cell == x_obs) ? 0 : std::max(std::abs(x_cell - x_obs) - 1, 0);
                // int delta_y = (y_cell == y_obs) ? 0 : std::max(std::abs(y_cell - y_obs) - 1, 0);
                distance = map_utils::distance(x_cell, y_cell, x_obs, y_obs);
                // if(distance > global_width)
                // {
                //     ROS_INFO("distance from (%zd, %zd) to (%zd,%zd) = %f", x_obs, y_obs, x_cell, y_cell, distance);
                // }
            }
        }
    };

    std::vector<double> distances;

    void enqueueObstacleCells(const nav_msgs::OccupancyGrid& map, std::priority_queue<DistNode>& search_queue);

    void expandNode(const DistNode& node, std::priority_queue<DistNode>& search_queue);
    

  public:

    DistanceGrid(const nav_msgs::OccupancyGrid&);
   

    float operator[](size_t idx) const {return distances[idx];}

    void setDistances(const nav_msgs::OccupancyGrid& map);

};

}

#endif
