#include <ros/ros.h>
#include <ros/console.h>
#include <finderbot/mapping.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

ros::Publisher map_publisher;
mapping::MiniMapper* mini_map_builder;

void handleLaserScan(const sensor_msgs::LaserScan laser_msg)
{
    //grow map data based on scan
    mini_map_builder->buildMap(laser_msg);
    //publish map data
    map_publisher.publish(mini_map_builder->getMap());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mini_mapping");
    ros::NodeHandle nh;

    double map_width;
    double map_height;
    double map_resolution;
    std::string local_frame_id;
    nh.param<double>("mini_map_width", map_width, 200);
    nh.param<double>("mini_map_height", map_height, 200);
    nh.param<double>("map_resolution", map_resolution, 0.020);
    nh.param<std::string>("local_frame_id", local_frame_id, "laser_frame");
  
    mini_map_builder = new mapping::MiniMapper(map_width, map_height, map_resolution, local_frame_id);

    ros::Subscriber laserHandler = nh.subscribe<sensor_msgs::LaserScan>(
                                                            "/hokuyo_data",
                                                            1,
                                                            handleLaserScan
                                                            );
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("slam_map",1,true);

    ros::spin();
}  