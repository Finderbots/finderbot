#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/planner.h>
#include <vector>
#include <queue>
#include <planner.h>


class DistanceGrid
{
    std::vector<DistNode> distances;
    size_t global_width;
    size_t global_height;

    struct DistNode
    {
        double distance;
        size_t obstacle_idx;

        bool operator<(const DistNode& rhs) const
        {
            return distance < rhs.distance;
        }

        DistNode(size_t cell_idx, obstacle_idx)
        : obstacle_idx(obstacle_idx)
        {
            if (cell_idx == obstacle_idx) distance = 0;

            else
            {
                x_cell = map_utils::rowFromOffset(cell_idx, global_width);
                y_cell = map_utils::colFromOffset(cell_idx, global_width);
                x_obs = map_utils::rowFromOffset(obstacle_idx, global_width);
                y_obs = map_utils::colFromOffset(obstacle_idx, global_width);

                // int delta_x = (x_cell == x_obs) ? 0 : std::max(std::abs(x_cell - x_obs) - 1, 0);
                // int delta_y = (y_cell == y_obs) ? 0 : std::max(std::abs(y_cell - y_obs) - 1, 0);

                distance = map_utils::distance(x_cell, y_cell, x_obs, y_obs);
            }
        }
    };

  public:

    void setCellDist(size_t cell_idx, obstacle_idx)
    {
        if (cell_idx == obstacle_idx) distances[cell_idx] = 0;
        else
        {
            x_cell = map_utils::rowFromOffset(cell_idx, global_width);
            y_cell = map_utils::colFromOffset(cell_idx, global_width);
            x_obs = map_utils::rowFromOffset(obstacle_idx, global_width);
            y_obs = map_utils::colFromOffset(obstacle_idx, global_width);

            // int delta_x = (x_cell == x_obs) ? 0 : std::max(std::abs(x_cell - x_obs) - 1, 0);
            // int delta_y = (y_cell == y_obs) ? 0 : std::max(std::abs(y_cell - y_obs) - 1, 0);

            distances[cell_idx] = map_utils::distance(x_cell, y_cell, x_obs, y_obs);
        }
    }

    void setDistances(const std::vector<double>& map)
    {
        assert(map.size() == distances.size());

        for (size_t i = 0; i < map.size(); i++)
        {
            distances[i].distance = std::numeric_limits<double>::max();
        }

        std::priority_queue<DistNode> search_queue;
        //TODO Finish this function
        enqueueObstacleCells(map, search_queue);

        while (!search_queue.empty())
        {
            DistNode next_node = search_queue.top()
            search_queue.pop();

            if (next_node.distance < )
        }
    }

    void enqueueObstacleCells(const std::vector<double>& map, std::priority_queue<DistNode>& search_queue)
    {
        for (size_t i = 0; i < map.size(); i++)
        {
            //if most likely an obstacle, treat it like an obstacle
            if (map[i] >= 50)
            {
                //if obstacle found then expand and add to searchQueue
                expandFromObstacle(distances[i], search_queue);
            }

        }
    }

    void expandFromObstacle(const DistNode& node, std::priority_queue<DistNode>& search_queue)
    {
        const int x_deltas[4] = {1,-1, 0, 0};
        const int y_deltas[4] = {0, 0, 1,-1};

        size_t x = rowFromOffset(obstacle_idx, global_width);
        size_t y = colFromOffset(obstacle_idx, global_width);

        for (int i = 0; i < 4; i++)
        {
            //if adjaacent cell is in map
            if (pointInMap(x + x_deltas[i], y+y_deltas[i], global_width, global_height))
            {
                //get neighbor_idx
                size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i],
                                                                 y + y_deltas[i],
                                                                 global_width);
                //create dist node relative to this obstacle
                DistNode adjacent_node(neighbor_idx, node.obstacle_idx);

                //if this obstacle is closer than the one previously assigned to
                //that cell then replace the dist node and add the new node to
                //the searchQueue
                if (adjacent_node.distance < distances[neighbor_idx].distance)
                {
                    distances[neighbor_idx] = adjacent_node;
                    search_queue.push(adjacent_node);
                }
            }
        }
    }

};






//cell is a frontier if it is unexplored (-1) and neighbor is free (0)
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

std::vector<size_t> growFrontier(const size_t cell, const std::vector<double>& map,
                                                std::set<size_t>& visited_frontiers)

{
    std::queue <size_t> cell_queue;
    cell_queue.push(cell);
    visited_frontiers.insert(cell);

    const int num_neighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };

    std::vector<size_t> frontier;

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
    {

    return frontier;
}

std::vector<size_t> pathToFrontier(const std::vector<size_t>& frontier,
                                   const size_t robot_pose_idx
                                   const std::vector<double>& map,
                                   const Planner& planner
                                   )
    assert(!frontier.empty());

    //sort frontier so that the cell that is furthest from all the others is first

    //loop through sorted vector, find path to that point

    //if path is valid, then return the first one

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

    Planner.aStar()
}

std::vector<std::vector<size_t> > getMapFrontiers(const std::vector<double>& map,
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
        size_t x = map_utils::rowFromOffset(next_idx, global_width);
        size_t y = map_utils::colFromOffset(next_idx, global_width);

        for (int n = 0; n < num_neighbors; ++n)
        {
            neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y+y_deltas[i]);

            if (visited_idxs.find() != visited_idxs.end() ||
                                !map_utils::pointInMap(neighbor_idx, global_width, map.size()/global_width)
            {
                continue;
            }

            else if (isFrontier(neighbor_idx, map))
            {
                std::vector<size_t> f growFrontier(neighbor_idx, map, visited_idxs);

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
