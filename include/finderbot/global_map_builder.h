#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>
#include <cassert>

namespace global_mapping
{

class GlobalMapBuilder
{
    nav_msgs::OccupancyGrid map_;
    std::vector<double> log_odds_map_;

    std::string laser_frame_id_;
    std::string world_frame_id_;

    tf::TransformListener tf_listener_;
    ros::Publisher map_publisher_;

    double x_init_;
    double y_init_;

    size_t init_map_x_;
    size_t init_map_y_;

    size_t map_x_;
    size_t map_y_;

    bool transform_initialized;

    void updatePosition();

public:

    GlobalMapBuilder(int width, int height, double resolution, std::string local_frame_id);

    void addLocalMap(const nav_msgs::OccupancyGrid& local_map);

    const nav_msgs::OccupancyGrid& getMap()
    {
        return map_;
    }

    const std::vector<double>& getLogOdds()
    {
        return log_odds_map_;
    }

};

inline size_t getOffsetRowCol(size_t row, size_t col, size_t ncol)
{
    return (row * ncol) + col;
}

}