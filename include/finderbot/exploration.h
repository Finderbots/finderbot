#include <finderbot/planner.h>
#include <finderbot/DistanceGrid.h>
#include <vector>
#include <queue>
#include <planner.h>

struct frontier_t 
{
    std;;vector<size_t> idxs;
};

bool isFrontier(const size_t idx, const std::vector<double>& map);

std::vector<size_t> growFrontier(const size_t cell, const std::vector<double>& map,
                                                std::set<size_t>& visited_frontiers);


std::vector<size_t> pathToFrontier(const std::vector<size_t>& frontier,
                                   const size_t robot_pose_idx
                                   const std::vector<double>& map,
                                   const Planner& planner
                                   );

std::vector<size_t> exploreFrontier(Planner& planner);


size_t nearestNavigableCell(size_t robot_pose_idx,
                               size_t desired_idx,
                               const std::vector<double> map,
                               const Planner& planner);

size_t nearestFreeCell();


std::vector<std::vector<size_t> > findMapFrontiers(const std::vector<double>& map,
                                                    size_t robot_pose_idx,
                                                    double min_dist_to_frontier);




