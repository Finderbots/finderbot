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
    std::vector<size_t> path_coordinates_;

    distance_grid::DistanceGrid obstacle_distance_map_;

    // Output command velocities based on plan
    ros::Publisher command_velocities_;

    size_t source_row;
    size_t source_col;
    // INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
    //          an x,y destination
    // OUTPUT:  command velocities... angular and linear velocities
    //      geometry_msgs/Twist.h
    //      at any given time you can only rotate or go forward or backward
  public:
    Planner(std::vector<double> global_map);

    std::vector<size_t> * aStar(size_t goal_row, size_t goal_col);

    void getNeighbors(const Node & node, std::vector<Node*> & neighbors);

    // This handles request and service stuff now
    std::vector<size_t> * getPath(Node * goal);

    bool isValidNeighbor(size_t idx);

    void setPose(size_t row, size_t col);

    const size_t getPoseIdx()
    {
        return map_utils::getOffsetRowCol(source_row, source_col, global_width);
    }

    bool isGoal(Node * node, size_t goal_row, size_t goal_col);

    // bool pathCb(finderbot::getPath::Request  &req,
    //             finderbot::getPath::Response &res);

    double distAt(size_t idx)
    {
        return obstacle_distance_map_[idx];
    }

    const std::vector<double>* getMapPtr()
    {
        return &global_map_;
    }
};

class Compare {
  public:
    bool operator() (const Node * node1, const Node * node2) {
        return node1->f_score > node2->f_score;
    }
};


