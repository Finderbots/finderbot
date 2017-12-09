#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <finderbot/global_map_builder.h>
#include <finderbot/UWB_data.h>
#include <finderbot/PF_Input.h>
#include <finderbot/Pose.h>
#include <finderbot/GetMap.h>


#include <string>
#include <cassert>

ros::Publisher global_map_publisher;
ros::Publisher local_map_publisher;
ros::Publisher pf_publisher;

global_mapping::GlobalMapBuilder* global_map_builder;

void handleUWB(const finderbot::UWB_data uwb)
{
    if (uwb.vital)
    {
        std::vector<size_t> uwb_ray;

        bool ret = global_map_builder->castRayToObstacle(uwb.angle, uwb.distance, uwb_ray);
        
        if (ret)
        {
            global_map_builder->add_ubw(uwb_ray.back());
        }
    }
}

void handleLaserScan(const sensor_msgs::LaserScan scan)
{
    if (!global_map_builder->initialized()) return;

    global_map_builder->buildMapFromScan(scan);
    pf_publisher.publish(global_map_builder->getPFData());
    global_map_publisher.publish(global_map_builder->getGlobalMap());
    local_map_publisher.publish(global_map_builder->getLocalMap());
}

void handlePose(const finderbot::Pose new_pose)
{
    global_map_builder->updatePose(new_pose);
}

bool sendMap(finderbot::GetMap::Request& req, finderbot::GetMap::Response& res)
{
    for (size_t i = 0; i < global_map_builder->getGlobalMap().data.size(); i++)
    {
        res.map.push_back(global_map_builder->getGlobalMap().data[i]);
    }
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_mapping");
    ros::NodeHandle nh;

    int global_map_width;
    int global_map_height;
    int local_map_width;
    int local_map_height;
    double map_resolution_param;
    std::string local_frame_id;

    int global_map_length_m = 15;
    int local_map_length_m = 5;
    double map_resolution = 0.05;

    int default_global_cells = global_map_length_m / map_resolution;
    int default_local_cells = local_map_length_m / map_resolution;

    // ROS_INFO("GLOBAL MAP length: %d", default_global_cells);
    // ROS_INFO("LOCAL MAP length: %d", default_local_cells);

    nh.param<int>("global_map_width", global_map_width, default_global_cells);
    nh.param<int>("local_map_width", local_map_width, default_local_cells);

    nh.param<double>("map_resolution", map_resolution_param, map_resolution);
    nh.param<std::string>("local_frame_id", local_frame_id, "laser_frame");

    ROS_INFO("GLOBAL MAP: width = %d", global_map_width);
    global_map_builder = new global_mapping::GlobalMapBuilder(global_map_width, 
                                                              global_map_width,
                                                              local_map_width,
                                                              local_map_width, 
                                                              map_resolution_param, 
                                                              local_frame_id);


    ros::Subscriber local_map_handler = nh.subscribe<sensor_msgs::LaserScan>("hokuyo_data", 1, handleLaserScan);

    ros::Subscriber uwb_handler = nh.subscribe<finderbot::UWB_data> ("UWB_data", 1, handleUWB);

    ros::Subscriber pose_handler = nh.subscribe<finderbot::Pose>("finderbot_pose", 1, handlePose);
    global_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);

    local_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);

    pf_publisher = nh.advertise<finderbot::PF_Input>("SLAM_pf", 1, true);

    ros::ServiceServer service = nh.advertiseService("get_finderbot_global_map", sendMap);

    std::cout << "GLOBAL MAP: spinning" << std::endl;
    ros::spin();
}
