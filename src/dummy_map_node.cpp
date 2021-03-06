#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <finderbot/GetMap.h>


#include <string>
#include <cassert>

ros::Publisher global_map_publisher;
ros::Publisher local_map_publisher;
ros::Publisher pf_publisher;

global_mapping::GlobalMapBuilder* global_map_builder;

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
    ros::init(argc, argv, "dummy_map_node");
    ros::NodeHandle nh;

    int global_map_width;
    int global_map_height;
    int local_map_width;
    int local_map_height;
    double map_resolution;
    std::string local_frame_id;


    nh.param<int>("global_map_width", global_map_width, 1000);
    nh.param<int>("global_map_height", global_map_height, 1000);
    nh.param<int>("local_map_height", local_map_height, 200);
    nh.param<int>("local_map_width", local_map_width, 200);

    nh.param<double>("map_resolution", map_resolution, 0.020);
    nh.param<std::string>("local_frame_id", local_frame_id, "laser_frame");


    global_map_builder = new global_mapping::GlobalMapBuilder(global_map_width, 
                                                              global_map_height,
                                                              local_map_width,
                                                              local_map_height, 
                                                              map_resolution, 
                                                              local_frame_id);

    ros::Subscriber local_map_handler = nh.subscribe<sensor_msgs::LaserScan>("hokuyo_data", 1, handleLaserScan);

    global_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);

    local_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);

    ros::ServiceServer service = nh.advertiseService("get_finderbot_global_map", sendMap);

    std::cout << "GLOBAL MAP: spinning" << std::endl;
    ros::spin();
}