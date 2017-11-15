#ifndef MAPPING_H_
#define MAPPING_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h> 
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <finderbot/ray_caster.h>

namespace mapping
{

class MiniMapper
{
    bool updateMap(const sensor_msgs::LaserScan& scan, long int dx, long int dy, double theta);
    bool castRayToObstacle(const nav_msgs::OccupancyGrid& map, 
                    double angle, double range, std::vector<size_t>& raycast);

    void updateOccupancyVal(bool occupied, size_t idx, std::vector<int8_t>& occupancy, std::vector<double>& log_odds) const;
    
    inline void vec_updateOccupancy(bool occupied, std::vector<size_t>& indices, std::vector<int8_t>& occupancy, std::vector<double>& log_odds) const
    {
        std::vector<size_t>::const_iterator it = indices.begin();
        for (; it != indices.end(); ++it)
        {
            updateOccupancyVal(occupied, *it, occupancy, log_odds);
        }
    }

    double angle_resolution_;
    double p_occupied_laser_;
    double p_occupied_no_laser_;
    double large_log_odds_;
    double max_log_odds_belief_;
    double x_init_;
    double y_init_;
    bool has_frame_id_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string world_frame_id_;
    std::string map_frame_id_;
    long int prev_map_x_;
    long int prev_map_y_;
    nav_msgs::OccupancyGrid map_;
    std::vector<double> log_odds_; //log_odd = log(p(x)/ (1- p(x)))
    ray_caster::RayCaster ray_caster_;



public:
    MiniMapper(int width, int height, double resolution);

    void buildMap(const sensor_msgs::LaserScan& scan);

    nav_msgs::OccupancyGrid getMap() const
    {
        return map_;
    }

};

inline int offsetRowCol(int row, int col, size_t ncol)
{
  return (row * ncol) + col;
}

}
#endif