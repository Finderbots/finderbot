#include <finderbot/exploration.h>


//cell is a frontier if it is unexplored (-1) and neighbor is free (0)
bool isFrontier(const size_t idx, const nav_msgs::OccupancyGrid* map)
{
    //using prob map not log odds
    if (map == nullptr){
        std::cout <<"EXPLORE: isFrontier map is null";
        return false;
    }
    size_t x = map_utils::rowFromOffset(idx, map->info.width);
    size_t y = map_utils::colFromOffset(idx, map->info.width);

    if (!map_utils::pointInMap(x, y, map->info.width, map->info.height))
    {
        return false;
    }

    if(map->data[idx] != -1)
    {
        return false;
    }

    const int num_neighbors = 4;
    const int x_deltas[] = {-1, 1, 0, 0};
    const int y_deltas[] = {0, 0, 1, -1};

    //if the point is unexplored and neighbors an explored space, then it is a frontier cell
    for (int i = 0; i < num_neighbors; i++)
    {
        size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y+y_deltas[i], map->info.width);
        if (map->data[neighbor_idx] == 0)
        {
            return true;
        }
    }

    return false;
}

void growFrontier(const size_t cell, 
                  const nav_msgs::OccupancyGrid* map,
                  frontier_t& frontier,
                  std::set<size_t>& visited_frontiers)

{
    if (map == nullptr){
        std::cout <<"EXPLORE: growFrontier map is null";
        return;
    }
    std::queue <size_t> cell_queue;
    cell_queue.push(cell);
    visited_frontiers.insert(cell);

    const int num_neighbors = 8;
    const int x_deltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int y_deltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };

    

    while (!cell_queue.empty())
    {
        size_t next_cell = cell_queue.front();
        cell_queue.pop();

        frontier.idxs.push_back(next_cell);

        size_t x = map_utils::rowFromOffset(next_cell, map->info.width);
        size_t y = map_utils::colFromOffset(next_cell, map->info.width);

        for (int i = 0; i < num_neighbors; i++)
        {
            size_t neighbor_idx = map_utils::getOffsetRowCol(x + x_deltas[i], y + y_deltas[i],
                                                    map->info.width);

            //if not explored already  and is frontier then add to this frontier
            if (visited_frontiers.find(neighbor_idx) == visited_frontiers.end()
                    && isFrontier(neighbor_idx, map))
            {
                visited_frontiers.insert(neighbor_idx);
                cell_queue.push(neighbor_idx);
            }
        }
    }
}

std::vector<size_t>* pathToFrontier(frontier_t& frontier,
                                   Planner& planner
                                   )
{
    assert(!frontier.idxs.empty());
    const nav_msgs::OccupancyGrid* map = planner.getMapPtr();

    if (map == nullptr){
        std::cout <<"EXPLORE: pathToFrontier map is null";
        return nullptr;
    }
    size_t robot_pose_idx = planner.getPoseIdx();

    //sort frontier so that the cell that is furthest from obstacles is first
    //frontiers order doesnt actually matter so no need to make a new one
    std::sort(frontier.idxs.begin(), 
              frontier.idxs.end(),
              [&planner](size_t lhs, size_t rhs) -> 
              bool {
                return planner.distAt(lhs) > planner.distAt(rhs); 
              });


    //go through vector return first valid path
    for (size_t i = 0; i < frontier.idxs.size(); i++)
    {

        size_t goal_x = map_utils::rowFromOffset(frontier.idxs[i], map->info.width);
        size_t goal_y = map_utils::colFromOffset(frontier.idxs[i], map->info.width);
        
        std::vector<size_t>* path = planner.aStar(goal_x, goal_y);

        if (path != nullptr)
        {
            return path;
        }
    }

    return nullptr;
}


void findMapFrontiers(const Planner& planner,
                 std::vector<frontier_t>& frontiers,
                 double min_dist_to_frontier)
{
    // std::cout << "ENTER findMapFrontiers" << std::endl;

    const nav_msgs::OccupancyGrid* map = planner.getMapPtr();
    if (map == nullptr){
        std::cout <<"EXPLORE: growFrontier map is null";
        return;
    }
    const size_t robot_pose_idx = planner.getPoseIdx();

    std::set<size_t> visited_idxs;
    std::queue<size_t> cellQueue;

    cellQueue.push(robot_pose_idx);
    visited_idxs.insert(robot_pose_idx);


    const int num_neighbors = 4;
    const int x_deltas[] = {-1, 1, 0, 0};
    const int y_deltas[] = {0, 0, 1, -1};

    size_t x = map_utils::rowFromOffset(robot_pose_idx, map->info.width);
    size_t y = map_utils::colFromOffset(robot_pose_idx, map->info.width);

    std::cout << "EXPLORE: growFRontierstart cell (" << x << ", " << y << ")" << std::endl;

    while (!cellQueue.empty())
    {
        size_t next_idx = cellQueue.front();
        cellQueue.pop();

        //TODO work these out for realz
        size_t x = map_utils::rowFromOffset(next_idx, map->info.width);
        size_t y = map_utils::colFromOffset(next_idx, map->info.width);

        // std::cout << "next cell (" << x << ", " << y << ")" << std::endl;

        for (int i = 0; i < num_neighbors; ++i)
        {
            size_t neighbor_x = x + x_deltas[i];
            size_t neighbor_y = y + y_deltas[i];

            size_t neighbor_idx = map_utils::getOffsetRowCol(neighbor_x, neighbor_y, map->info.width);
            // std::cout << "map[" << x << ", " << y << "] = " << (int)map->data[neighbor_idx] << std::endl;
            
            //continue if neighbor already visited or if not in map
            if (visited_idxs.find(neighbor_idx) != visited_idxs.end() 
                || !map_utils::pointInMap(neighbor_x, neighbor_y, map->info.width, map->info.height))
            {
                continue;
            }

            //if point is a frontier, build frontier vector and add to frontiers
            else if (isFrontier(neighbor_idx, map))
            {

                std::cout << "FRONTIER AT (" << neighbor_x << ", " << neighbor_y << ")" << std::endl;
                frontier_t f;

                growFrontier(neighbor_idx, map, f, visited_idxs);

                if (f.idxs.size() * map->info.resolution >= min_dist_to_frontier)
                {

                    frontiers.push_back(f);
                }
            }

            else if (map->data[neighbor_idx] < 50)
            {
                visited_idxs.insert(neighbor_idx);
                cellQueue.push(neighbor_idx);
            }
        }
    }

    // return frontiers;
}

//returns path to nearest frontier
// if no valid path, returns its own pose
std::vector<size_t> exploreFrontiers(Planner& planner, std::vector<frontier_t>& frontiers)
{
    //TODO actual min_dist_to_frontier
    // std::cout << "exploreFrontiers" << std::endl;
    size_t robot_pose_idx = planner.getPoseIdx();
    size_t global_width = planner.getMapPtr()->info.width;
    if (frontiers.empty())
    {
        std::vector <size_t> emptyPath;
        emptyPath.push_back(robot_pose_idx);
        return emptyPath;
    }

    std::vector<std::vector<size_t> > paths;

    //this iterates through frontiers,
    //calculates a path to point farthest from an obstacle
    //pushes that path to the end of paths
    std::transform(frontiers.begin(), frontiers.end(), std::back_inserter(paths),
        [&planner](frontier_t& frontier) {return *pathToFrontier(frontier, planner);});

    // std::cout << "global_width = " << global_width << std::endl;
    // std::cout << "GENERATED " << paths.size() << " PATHS" << std::endl;

    for (size_t i = 0; i < paths[0].size(); i++)
    {
        size_t x = map_utils::rowFromOffset(paths[0][i], global_width);
        size_t y = map_utils::colFromOffset(paths[0][i], global_width);

        std::cout << "(" << x << ", " << y << ")" << std::endl;

    }
    return *std::min_element(paths.begin(), paths.end(), 
            [](const std::vector<size_t>& lhs, const std::vector<size_t>& rhs)
            {
                return lhs.size() < rhs.size();
            });
}
