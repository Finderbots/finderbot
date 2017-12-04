#include <finderbot/path_execution.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
// subscribes to both the slam and the explore nodes
// might as well just have this be the explorer

//   gets the pose from slam
//   gets the destination from the explore node

//assumes can overwrite global map in planner
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;

    Executor path_executor("world", "laser_frame");

    ros::Subscriber global_map_handler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, &Executor::handleGlobalMap, &path_executor);
    ros::Subscriber pose_handler = nh.subscribe<finderbot::Pose>("finderbot_pose", 1, &Executor::handlePose, &path_executor);
    std::vector<frontier_t> frontiers;
    ROS_INFO("Execution Node Starting");

    while (ros::ok())
    {
        if (!path_executor.initialized()) {
            // ROS_INFO("Map Uninitialized: Continue");
            ros::spinOnce();
            continue;
        }
        // ROS_INFO("INITIALIZED");
        ros::spinOnce();

        frontiers.clear();
        findMapFrontiers(path_executor.getPlanner(), frontiers, 1);
        ROS_INFO("EXECUTE: FOUND %zd FRONTIERS", frontiers.size());

        if (frontiers.empty())
        {
            continue;
        }

        std::vector<size_t> path = exploreFrontiers(path_executor.getPlanner(), frontiers);

        ROS_INFO("EXECUTE PATH OF LENGTH %zd", path.size());
        path_executor.pathExecution(path);
    }

    return 0;
}
