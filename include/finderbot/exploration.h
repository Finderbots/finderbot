#ifndef EXPLORATION_H_
#define EXPLORATION_H_

#include <finderbot/planner.h>
#include <finderbot/DistanceGrid.h>
#include <vector>
#include <queue>

struct frontier_t 
{
    std::vector<size_t> idxs;
};

// extern nav_msgs::OccupancyGrid G_FRONTIER_MAP;

//planner stores map and initialized obstacle distance grid
std::vector<size_t> exploreFrontiers(Planner& planner, std::vector<frontier_t>& frontiers, size_t min_dist_to_frontier);




void findMapFrontiers(const Planner& planner,
                      std::vector<frontier_t>& frontiers,
                      double min_frontier_len);

#endif