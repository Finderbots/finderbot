#ifndef PATH_EXECUTION_H_
#define PATH_EXECUTION_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/planner.h>
#include <finderbot/map_utils.h>
#include <finderbot/Pose.h>
#include <geometry_msgs/Twist.h>
#include <finderbot/uwb.h>
//#include <finderbot/UWB/ModuleConnector.hpp>
#include <ctime>
#include <iostream>
#include <unistd.h>


#include <math.h>
#include <string>


class Executor
{
    size_t current_row_;
    size_t current_col_;

    double x_init_;
    double y_init_;

    size_t init_map_x_;
    size_t init_map_y_;

    double current_theta_;

    bool map_initialized_ = false;
    bool pose_initialized_ = false;
    bool moving_to_point_ = false;

    Planner* planner_;

    geometry_msgs::Twist cmd_;
    std::string world_frame_id_;
    std::string local_frame_id_;

    ros::NodeHandle nh;
    ros::Publisher command_velocities_pub_;

    bool turnTheta(double goal_theta);
    void drive();

    bool thetaCloseEnough(double threshold, double goal_theta);
    void goToNextNodeInPath(size_t goal_row, size_t goal_col);
    bool closeEnoughToGoal(double radius, int goal_row, int goal_col);

  public:
    ros::ServiceClient client_;
<<<<<<< HEAD
    // finderbot::UWBScan srv;
=======
    //finderbot::UWBScan srv;
>>>>>>> 848cddc64cd668e13c80dbc32871471f6c08d619
    Executor(std::string world_frame, std::string local_frame);

    void pathExecution(std::vector<size_t>& path);
    void handleGlobalMap(const nav_msgs::OccupancyGrid);
    void handlePose(const finderbot::Pose);

    Planner& getPlanner() {return *planner_;}
    bool initialized() {return pose_initialized_;}
    size_t getCurrRow() {return current_row_;}
    size_t getCurrCol() {return current_col_;}
	// ~Executor();
	
};
#endif
