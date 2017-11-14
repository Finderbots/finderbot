#include <ros/ros.h>
#include <finderbot/mapping.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

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
    ros::NodeHandle nh("~");

    double map_width;
    double map_height;
    double map_resolution;
    nh.param<double>("mini_map_width", map_width, 200);
    nh.param<double>("mini_map_height", map_height, 200);
    nh.param<double>("map_resolution", map_resolution, 0.020);
  
    mini_map_builder = new mapping::MiniMapper(map_width, map_height, map_resolution);

    ros::Subscriber laserHandler = nh.subscribe<sensor_msgs::LaserScan>(
                                                            "hokuyo_data",
                                                            1,
                                                            handleLaserScan
                                                            );
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("slam_map",1,true);

    ros::spin();
}  