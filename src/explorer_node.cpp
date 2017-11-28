#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

//cell is a frontier if it is unexplored (-1) and neighbor is free (0)
bool isFrontier(size_t idx, const nav_msgs::OccupancyGrid& map)
{
    //using prob map not log odds
    //point must be in map, and 
    if (!pointInMap(idx, map.info.width, map.info.height) || map.data[idx] != -1)
    {
        return false;
    }

    const int num_neighbors = 4;
    const int x_deltas[] = {-1, 1, 0, 0};
    const int y_deltas[] = {0, 0, 1, -1};

    for (int i = 0; i < num_neighbors; i++)
    {
        neighbor_idx = getOffsetRowCol(x + x_deltas[i], y+y_deltas[i]);
        if (map.data[neighbor_idx] == 0)
        {
            return true;
        }
    }

    return false;
}

std::vector<size_t> grow_frontier(size_t cell, const nav_msgs::OccupancyGrid& map, 
                                                std::set<size_t>& visited_frontiers)

{
    std::queue <size_t> cell_queue;
    cell_queue.push(cell);
    visited_frontiers.insert(cell);

    const int num_neighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };

    std::vector<size_t> frontier;

    
}                                                                                                

std::vector<std::vector<size_t> > getMapFrontiers(const nav_msgs::OccupancyGrid& map, 
                                                    size_t robot_pose_idx,
                                                    double min_dist_to_frontier)
{
    std::vector<std::vector<size_t> > frontiers;
    std::set<size_t> visited_idxs;

    std::queue<size_t> cellQueue;
    cellQueue.push(robot_pose_idx);
    visited_idxs.insert(robot_pose_idx);

    const int num_neighbors = 4;
    const int x_deltas[] = {-1, 1, 0, 0};
    const int y_deltas[] = {0, 0, 1, -1};

    while (!cellQueue.empty())
    {
        size_t next_idx = cellQueue.front();
        cellQueue.pop();

        //work these out for realz
        size_t x = getRowOffset();
        size_t y = getColOffset();

        for (int n = 0; n < num_neighbors; ++n)
        {
            neighbor_idx = getOffsetRowCol(x + x_deltas[i], y+y_deltas[i]);

            if (visited_idxs.find() != visited_idxs.end() || 
                                !pointInMap(neighbor_idx)
            {
                continue;
            }

            else if (isFrontier(neighbor_idx, map))
            {
                std::vector<size_t> f grow_frontier(neighbor_idx, map, visited_idxs);

                if (f[0].size() * map.info.resolution >= min_dist_to_frontier)
                {
                    frontiers.push_back();
                }
            }

            else if (map.data[neighbor_idx] < 0)
            {
                visited_idxs.insert(neighbor_idx);
                cellQueue.push(neighbor_idx);
            }
        }
    }
    return frontiers;
}