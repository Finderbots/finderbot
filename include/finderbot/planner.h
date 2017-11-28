/* Finderbots: Implementation of A-star algorithm */
#include <limits.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <queue>
#include <stdlib.h>     /* atoi */
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "finderbot/getPath.h"
// #include <finderbot/global_map_builder.h>
// #include <finderbot/NewHits.h>

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
#include <cassert>

#define global_width 1000

struct Node
{
    int row;
    int col;
    int dist;
    struct Node * parent;
    bool visited;
    int g_score;
    int h_score;
    int f_score;
};

class Planner {
    // probabilities of obstacles in map
    std::vector<double> global_map_;

    // Nodes each representing a coordinate in the map
    std::vector<Node> nodes_;

    // A 2-D vector with the coordinates of each point in the map to visit
    std::vector<int> path_coordinates_;
    // Output command velocities based on plan
    ros::Publisher command_velocities_;

    // INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
    //          an x,y destination
    // OUTPUT:  command velocities... angular and linear velocities
    //      geometry_msgs/Twist.h
    //      at any given time you can only rotate or go forward or backward
  public:
    Planner(nav_msgs::OccupancyGrid global_map,
            std::vector< std::vector<int> > & path_coordinates,
            std::vector<Node> nodes);

    Node * aStar(int goal_row, int goal_col, int source_row, int source_col);

    void getNeighbors(const Node & node, std::vector<Node*> & neighbors);

    // This handles request and service stuff now
    void getPath(Node * goal);

    bool isGoal(Node * node, int goal_row, int goal_col);
};

class Compare {
  public:
    bool operator() (const Node * node1, const Node * node2) {
        return node1->f_score < node2->f_score;
    }
};

// Pass 2 nodes
inline int distance(const Node & node1, const Node & node2) {
    // Euclidean distance
    return sqrt(pow(node1.row-node2.row,2) + pow(node1.col-node2.col,2));
}

// Pass 2 sets of row and column
inline int distance(int row1, int col1, int row2, int col2) {
    // Euclidean distance
    return sqrt(pow(row1-row2,2) + pow(col1-col2,2));
}

/* Return true if the point lies in the map */
inline bool pointInMap(const int row, const int col)
{
    return ((0 <= col) && ((size_t) col < global_width) && (0 <= row) && ((size_t) row < global_map.info.height));
}

inline size_t getOffsetRowCol(size_t row, size_t col)
{
    return (row * global_width) + col;
}
