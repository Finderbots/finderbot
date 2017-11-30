#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/planner.h>
#include <finderbot/DistanceGrid.h>

#include <vector>
#include <queue>

//cell is a frontier if it is unexplored (-1) and neighbor is free (0)
struct frontier_t
{
    std::vector<size_t> cells;
};

struct robot_path_t
{
    std::vector<size_t> cells;
};

bool isFrontier(const size_t idx, const std::vector<double>& map)
{
    //using prob map not log odds
    //point must be in map, and
    if (!pointInMap(idx, global_width, map.size()/ global_width) || map[idx] != -1)
    {
        return false;
    }

    const int num_neighbors = 4;
    const int x_deltas[] = {-1, 1, 0, 0};
    const int y_deltas[] = {0, 0, 1, -1};

    for (int i = 0; i < num_neighbors; i++)
    {
        neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y+y_deltas[i], global_width);
        if (map[neighbor_idx] == 0)
        {
            return true;
        }
    }

    return false;
}

frontier_t growFrontier(const size_t cell, const std::vector<double>& map,
                                                std::set<size_t>& visited_frontiers)

{
    std::queue <size_t> cell_queue;
    cell_queue.push(cell);
    visited_frontiers.insert(cell);

    const int num_neighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };

    frontier_t frontier;

    while (!cell_queue.empty())
    {
        size_t next_cell = cell_queue.front();
        cell_queue.pop();

        frontier.push_back(next_cell);

        size_t x = getRowOffset(next_cell, global_width);
        size_t y = getColOffset(next_cell, global_width);

        for (int i = 0; i < num_neighbors; i++)
        {
            size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y + y_deltas[i],
                                                    global_width);
            if (visited_frontiers.find(neighbor_idx) == visited_frontiers.end()
                    && isFrontier(neighbor_idx))
            {
                visited_frontiers.insert(neighbor_idx);
                cell_queue.push(neighbor_idx);
            }

        }
    }

    return frontier;
}


robot_path_t pathToFrontier(const std::vector<size_t>& frontier,
                                   const size_t robot_pose_idx
                                   const std::vector<double>& map,
                                   const Planner& planner
                                   )
    assert(!frontier.empty());

    //sort frontier so that the cell that is furthest from all the others is first
    DistanceGrid dist = planner.getObstactleGrid();

    //loop through sorted vector, find path to that point
    std::sort(frontier.cells.begin(), frontier.cells.end(), [&dist](size_t lhs, size_t rhs) -> bool {
        return dist[lhs] > dist[rhs];
    });

    for (size_t i = 0; i < frontier.cells.size(); i++)
    {
        robot_path_t path = planner.aStar(robot_pose_idx, 
                                        getNearestNavigableCell(robot_pose_idx, frontier.cells[i], map, planner));

        if (path.cells.size() > 1)
        {
            return path;
        }
    }

    robot_path_t invalid;
    invalid.cells.push_back(robot_pose_idx);
    return invalid;
}

robot_path_t planPathToFrontier(const std::vector<std::vector<size_t> >& frontiers,
                                       const size_t robot_pose_idx,
                                       const std::vector<double>& map,
                                       const Planner& planner)
{
    if (frontiers.empty())
    {
        std::vector<size_t> empty_path;
        empty_path.push_back(robot_pose_idx);
        return empty_path;
    }   

    std::vector<robot_path_t > paths;

    //iterates through frontiers gets path and appends to path
    std::transform(frontiers.begin(), 
                   frontiers.end(), 
                   std::back_inserter(paths),
                   [&robotPose, &map, &planner](const frontier_t& f) {
        return path_to_frontier(f, robotPose, map, planner);
    });

    //return path in paths with lowest size
    return *std::min_element(paths.begin(), paths.end(), [](const robot_path_t& lhs, const robot_path_t& rhs){
        return lhs.size() < rhs.size();
    });
}

size_t searchToNearestFreeSpace(size_t pose_idx, const std::vector<double>& map, const Planner& planner)
{
    std::queue<size_t> cell_queue;
    cell_queue.push(pose_idx);

    std::set<size_t> visited_idxs;
    visited_idxs.insert(pose_idx);

    const int num_neighbors = 8;
    const int x_deltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int y_deltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };

    while (!cell_queue.empty())
    {
        size_t next_idx = cell_queue.front();
        cell_queue.pop();

        size_t x =      map_utils::rowFromOffset(robot_pose_idx, global_width);
        size_t y =      map_utils::colFromOffset(robot_pose_idx, global_width);

        if (map_utils::pointInMap(x,y, global_height, global_width))
        {
            return next_idx;
        }

        for (int i = 0; i < num_neighbors; i++)
        {
            size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y+y_deltas[i]);

            if (visited_idxs.find(neighbor_idx) != visited_idxs.end())
            {
               continue;
            }

            else if (map[neighbor_idx] < 50)
            {
                cell_queue.push(neighbor_idx);
                visited_idxs.push(neighbor_idx);
            }
        }
    }

    //couldnt find a valid spot to move to return original pose as error
    return pose_idx;
}

size_t getNearestNavigableCell(size_t robot_pose_idx,
                               size_t desired_idx,
                               const std::vector<double> map,
                               const Planner& planner)
{
    size_t goal_x = map_utils::rowFromOffset(desired_idx, global_width);
    size_t goal_y = map_utils::colFromOffset(robot_pose_idx, global_width);
    size_t x =      map_utils::rowFromOffset(robot_pose_idx, global_width);
    size_t y =      map_utils::colFromOffset(robot_pose_idx, global_width);

    robot_path_t path = Planner.aStar(robot_pose_idx, desired_idx);

    for (size_t i = 0; i < path.cells.size(); i++)
    {
        if (map_utils::pointInMap(idx))
        {
            return path.cells[i];
        }
    }

    return -1;
}

size_t 

std::vector<frontier_t> findMapFrontiers(const std::vector<double>& map,
                                                    size_t robot_pose_idx,
                                                    double min_dist_to_frontier)
{
    std::vector<frontier_t > frontiers;
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
        size_t x = map_utils::rowFromOffset(next_idx, global_width);
        size_t y = map_utils::colFromOffset(next_idx, global_width);

        for (int n = 0; n < num_neighbors; ++n)
        {
            size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y+y_deltas[i]);

            if (visited_idxs.find() != visited_idxs.end() ||
                                !map_utils::pointInMap(neighbor_idx, global_width, map.size()/global_width)
            {
                continue;
            }

            else if (isFrontier(neighbor_idx, map))
            {
                frontier_t f  = growFrontier(neighbor_idx, map, visited_idxs);

                //TODO: this wont compile
                if (f.cells.size() * resolution >= min_dist_to_frontier)
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
