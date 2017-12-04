#include <finderbot/path_execution.h>
#include <finderbot/exploration.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <finderbot/map_utils.h>
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
    ros::Publisher front_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_map", 1, true);
    std::vector<frontier_t> frontiers;
    ROS_INFO("Execution Node Starting");

    nav_msgs::OccupancyGrid front_map;

    while (ros::ok())
    {
        if (!path_executor.initialized()) {
            // ROS_INFO("Map Uninitialized: Continue");
            ros::spinOnce();
            continue;
        }
        // ROS_INFO("INITIALIZED");
        ros::spinOnce();
        front_map.info = path_executor.getPlanner().getMapPtr()->info;

        frontiers.clear();
        findMapFrontiers(path_executor.getPlanner(), frontiers, 3);
        std::cout << "done with frontiers" << std::endl;


        front_map.data.assign(front_map.info.width*front_map.info.height, -1);
        for (size_t i = 0; i < frontiers.size(); i++)
        {
            for (size_t j =0; j < frontiers[i].idxs.size(); j++)
            {
                size_t idx = frontiers[i].idxs[j];
                front_map.data[idx] = 0;

                // ROS_INFO("front at %zd)", idx);
            }
        }

        // std::cout << "frontier_map [0] = " << G_FRONTIER_MAP.data[0] << std::endl;
        front_pub.publish(front_map);

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
