#include <finderbot/DistanceGrid.h>

namespace distance_grid{

size_t DistanceGrid::global_width = 0;
size_t DistanceGrid::global_height = 0;

DistanceGrid::DistanceGrid(size_t global_width_, size_t global_height_)
{
    global_width = global_width_;
    global_height = global_height_;

    distances.resize(global_width * global_height);
}


void DistanceGrid::setDistances(const std::vector<double>& map)
{
    assert(map.size() == distances.size());


    for (size_t i = 0; i < map.size(); i++)
    {
        distances[i] = std::numeric_limits<double>::max();
    }

    std::priority_queue<DistNode> search_queue;

    enqueueObstacleCells(map, search_queue);


    while (!search_queue.empty())
    {
        DistNode next_node = search_queue.top();
        search_queue.pop();

        if (next_node.distance <= distances[next_node.cell_idx])
        {
            expandNode(next_node, search_queue);
        }
    }

}

void DistanceGrid::enqueueObstacleCells(const std::vector<double>& map, std::priority_queue<DistNode>& search_queue)
{
    for (size_t i = 0; i < map.size(); i++)
    {
        //if most likely an obstacle, treat it like an obstacle
        if (map[i] >= 50)
        {

            //if obstacle found then create distnode 
            //to expand around and add to searchQueue
            expandNode(DistNode(i,i), search_queue);
        }

    }
}

void DistanceGrid::expandNode(const DistNode& node, std::priority_queue<DistNode>& search_queue)
{
    const int x_deltas[4] = {1,-1, 0, 0};
    const int y_deltas[4] = {0, 0, 1,-1};

    size_t x = map_utils::rowFromOffset(node.cell_idx, global_width);
    size_t y = map_utils::colFromOffset(node.cell_idx, global_width);

    for (int i = 0; i < 4; i++)
    {

        //if adjaacent cell is in map
        if (map_utils::pointInMap(x + x_deltas[i], y+y_deltas[i], global_width, global_height))
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

            if (adjacent_node.distance < distances[neighbor_idx])
            {
                distances[neighbor_idx] = adjacent_node.distance;
                search_queue.push(adjacent_node);
            }
        }
    }
}



}
