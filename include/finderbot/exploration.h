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

// bool isFrontier(const size_t idx, const std::vector<double>& map);

// std::vector<size_t> growFrontier(const size_t cell, const std::vector<double>& map,
//                                                 std::set<size_t>& visited_frontiers);


// std::vector<size_t> pathToFrontier(const std::vector<size_t>& frontier,
//                                    const size_t robot_pose_idx
//                                    const std::vector<double>& map,
//                                    const Planner& planner
//                                    );

//planner stores map and initialized obstacle distance grid
std::vector<size_t> exploreFrontiers(Planner& planner, std::vector<frontier_t>& frontiers);


// size_t nearestNavigableCell(size_t desired_idx,
//                             const std::vector<double> map,
//                             const Planner& planner);

// size_t nearestFreeCell();


void findMapFrontiers(const Planner& planner,
                      const std::vector<frontier_t>& frontiers,
                      double map_resolution,
                      double min_dist_to_frontier);

#endif