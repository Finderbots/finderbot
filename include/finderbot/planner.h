/* Finderbots: Implementation of A-star algorithm */
#include <limits.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <queue>
#include <stdlib.h>     /* atoi */
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <finderbot/map_utils.h>

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
#include <cassert>

#include <finderbot/DistanceGrid.h>

#define global_width 1000
#define global_height 1000

class Planner {
    // probabilities of obstacles in map
    std::vector<double> global_map_;

    // Nodes each representing a coordinate in the map
    std::vector<Node> nodes_;
    
    // A 2-D vector with the coordinates of each point in the map to visit
    std::vector<int> path_coordinates_;

    distance_grid::DistanceGrid obstacle_distance_map_;

    // Output command velocities based on plan
    ros::Publisher command_velocities_;

    int source_row;
    int source_col;
    // INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
    //          an x,y destination
    // OUTPUT:  command velocities... angular and linear velocities
    //      geometry_msgs/Twist.h
    //      at any given time you can only rotate or go forward or backward
  public:
    Planner(std::vector<double> global_map);

    std::vector<int> * aStar(int goal_row, int goal_col);

    void getNeighbors(const Node & node, std::vector<Node*> & neighbors);

    // This handles request and service stuff now
    std::vector<int> * getPath(Node * goal);

    void setPose(int row, int col);

    bool isGoal(Node * node, int goal_row, int goal_col);

    // bool pathCb(finderbot::getPath::Request  &req,
    //             finderbot::getPath::Response &res);

    double distAt(size_t idx)
    {
        return obstacle_distance_map_[idx];
    }
};

class Compare {
  public:
    bool operator() (const Node * node1, const Node * node2) {
        return node1->f_score > node2->f_score;
    }
};


