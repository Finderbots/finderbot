#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>

static nav_msgs::OccupancyGrid global_map;
static tf::TransformListener* tf_listener;
ros::Publisher global_map_publisher;


inline size_t getOffsetRowCol(size_t row, size_t col)
{
    return (row * global_map.info.width) + col;
}


void handleLocalMap(const nav_msgs::OccupancyGrid local_map)
{
    //get transform from local_map
    tf::StampedTransform transform;
    try
    {
        // tf_listener->waitForTransform("world", "map_frame_id", local_map, ros::Duration(1));
        //get most recent transform
        tf_listener->lookupTransform("mini_mapping", "world", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    //assume map is coming in already in world orientation
    //need x, y to get offset
    const double x = transform.getOrigin().x();
    const double y = transform.getOrigin().y();

    //get row and column of 
    size_t row = x / global_map.info.resolution;
    size_t col = y / global_map.info.resolution;

    size_t map_array_offset = getOffsetRowCol(row, col);

    for (size_t i = 0; i < local_map.data.size(); i++)
    {
        global_map.data[i+map_array_offset] = local_map.data[i];
    }

    global_map_publisher.publish(global_map);
    //get 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_mapping");
    ros::NodeHandle nh;

    int map_width;
    int map_height;
    double map_resolution;

    nh.param<int>("map_width", map_width, 10000);
    nh.param<int>("map_height", map_height, 10000);
    nh.param<double>("map_resolution", map_resolution, 0.020);

    global_map.info.width = map_width;
    global_map.info.height = map_height;
    global_map.info.resolution = map_resolution;

    global_map.info.origin.position.x = -static_cast<double>(map_width) / 2*map_resolution;
    global_map.info.origin.position.y = -static_cast<double>(map_height) / 2*map_resolution;
    global_map.info.origin.orientation.w = 1.0;

    tf_listener = new tf::TransformListener;
    global_map.data.assign(map_width*map_height, -1);

    ros::Subscriber localMapHandler = nh.subscribe<nav_msgs::OccupancyGrid>("local_map", 1, handleLocalMap);

    global_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);

    ros::spin();
}