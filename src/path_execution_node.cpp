#include <finderbot/path_execution.h>
#include <finderbot/exploration.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <finderbot/map_utils.h>
#include <finderbot/UWBScan.h>
#include <iostream>
// subscribes to both the slam and the explore nodes
// might as well just have this be the explorer

//   gets the pose from slam
//   gets the destination from the explore node
//XeThru::Uwb * uwb_;
//ros::Publisher uwb_publisher;


//assumes can overwrite global map in planner
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;

    Executor path_executor("world", "laser_frame");

    ros::ServiceClient uwb_scan_client = nh.serviceClient<finderbot::UWBScan>("UWB_Scan");
    finderbot::UWBScan uwb_scan_srv;
    ros::Subscriber global_map_handler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, &Executor::handleGlobalMap, &path_executor);
    ros::Subscriber pose_handler = nh.subscribe<finderbot::Pose>("finderbot_pose", 1, &Executor::handlePose, &path_executor);
    ros::Publisher front_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_map", 1, true);
    std::vector<frontier_t> frontiers;
    ROS_INFO("Execution Node Starting");

    nav_msgs::OccupancyGrid front_map;

    ros::Rate rate(100);

    while (ros::ok())
    {
        if (!path_executor.initialized()) {
            // ROS_INFO("UNINITIALIZED EXECUTOR");
            ros::spinOnce();
            continue;
        }

        ROS_INFO("EXECUTE MAIN LOOP: spin");
        ros::spinOnce();
        front_map.info = path_executor.getPlanner().getMapPtr()->info;

        frontiers.clear();
        findMapFrontiers(path_executor.getPlanner(), frontiers, 3);


        front_map.data.assign(front_map.info.width*front_map.info.height, -1);
        for (size_t i = 0; i < frontiers.size(); i++)
        {
            for (size_t j =0; j < frontiers[i].idxs.size(); j++)
            {
                size_t idx = frontiers[i].idxs[j];
                front_map.data[idx] = 0;
            }
        }

        // std::cout << "frontier_map [0] = " << G_FRONTIER_MAP.data[0] << std::endl;

        ROS_INFO("EXECUTE: FOUND %zd FRONTIERS", frontiers.size());

        if (frontiers.empty())
        {
            // uwb_->uwb_stop();
            continue;
        }

        std::vector<size_t> path = exploreFrontiers(path_executor.getPlanner(), frontiers, 0);
        for (size_t i = 0; i < path.size(); i++)
        {
            ROS_INFO("(%zd, %zd)", map_utils::rowFromOffset(path[i], front_map.info.width),
                                   map_utils::colFromOffset(path[i], front_map.info.width));
            front_map.data[path[i]] = 100;
        }

        front_pub.publish(front_map);

        ROS_INFO("EXECUTE PATH OF LENGTH %zd", path.size());
        path_executor.pathExecution(path);

        // call UWB service here (how to get uwb object thru to service?)
        if (uwb_scan_client.call(uwb_scan_srv)) {
            ROS_INFO("Called UWB Scan Service");
        }
        else {
            ROS_ERROR("Failed to call UWB Scan Service");
            return 1;
        }

        // path_executor.srv.request.current_row = path_executor.getCurrRow();
        // path_executor.srv.request.current_col = path_executor.getCurrCol();
        // while (!path_executor.client_.call(path_executor.srv));


    }

    return 0;
}
