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

//planner stores map and initialized obstacle distance grid
std::vector<size_t> exploreFrontiers(Planner& planner, std::vector<frontier_t>& frontiers);




void findMapFrontiers(const Planner& planner,
                      std::vector<frontier_t>& frontiers,
                      double min_dist_to_frontier);

#endif