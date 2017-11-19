#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <finderbot/global_map_builder.h>

#include <string>
#include <cassert>

ros::Publisher global_map_publisher;
global_mapping::GlobalMapBuilder* global_map_builder;

void handleLocalMap(const nav_msgs::OccupancyGrid local_map)
{

    global_map_builder->addLocalMap(local_map);
    global_map_publisher.publish(global_map_builder->getMap());
    //get 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_mapping");
    ros::NodeHandle nh;

    int map_width;
    int map_height;
    double map_resolution;
    std::string local_frame_id;


    nh.param<int>("map_width", map_width, 10000);
    nh.param<int>("map_height", map_height, 10000);
    nh.param<double>("map_resolution", map_resolution, 0.020);
    nh.param<std::string>("local_frame_id", local_frame_id, "laser_frame");


    global_map_builder = new global_mapping::GlobalMapBuilder(map_width, 
                                                              map_height, 
                                                              map_resolution, 
                                                              local_frame_id);

    ros::Subscriber localMapHandler = nh.subscribe<nav_msgs::OccupancyGrid>("local_map", 1, handleLocalMap);

    global_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);

    ros::spin();
}