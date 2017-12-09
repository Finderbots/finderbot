#ifndef GLOBAL_MAP_BUILDER_H_
#define GLOBAL_MAP_BUILDER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>
#include <finderbot/PF_Input.h>
#include <finderbot/Pose.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>

#include <finderbot/ray_caster.h>
#include <finderbot/map_utils.h>

#include <vector>
#include <string>
#include <map>
#include <cassert>

namespace global_mapping
{

class GlobalMapBuilder
{
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::OccupancyGrid local_map_;
    std::vector<double> log_odds_map_;

    ray_caster::RayCaster ray_caster_;

    finderbot::PF_Input pf_;

    std::string laser_frame_id_;
    std::string world_frame_id_;

    // tf::TransformListener tf_listener_;
    ros::Publisher map_publisher_;

    size_t mini_map_width_;
    size_t mini_map_height_;

    double x_init_;
    double y_init_;
    double theta_init_;

    size_t init_map_x_;
    size_t init_map_y_;

    size_t map_x_;
    size_t map_y_;
    double theta_;

    double angle_resolution_;
    double p_occupied_laser_;
    double p_occupied_no_laser_;
    double max_log_odds_;
    double belief_threshold_;

    bool pose_initialized_;
    bool has_frame_id_;

    // void updatePosition();
    void updateProbOccupied(bool occupied, size_t idx);
    void updateLocalOccupancy(bool occupied, size_t idx);
    void vec_updateLocalOccupancy(bool occupied, std::vector<size_t>& ray);
    void addLocalMapToGlobal();
    void createNewLocalMap(const sensor_msgs::LaserScan& scan);


public:

    GlobalMapBuilder(int global_width, int global_height, 
                    int local_width, int local_height, 
                    double resolution, std::string laser_frame_id);

    bool castRayToObstacle(double angle, double range, std::vector<size_t>& raycast);

    void buildMapFromScan(const sensor_msgs::LaserScan& scan);

    void add_uwb(size_t pt)
    {
        global_map_.data[pt] = 101;
    }
    void updatePose(const finderbot::Pose& new_pose);

    bool initialized() {return pose_initialized_; }

    const nav_msgs::OccupancyGrid& getGlobalMap()
    {
        return global_map_;
    }

    const nav_msgs::OccupancyGrid& getLocalMap()
    {
        return local_map_;
    }
    const std::vector<double>& getLogOdds()
    {
        return log_odds_map_;
    }

    const finderbot::PF_Input& getPFData()
    {
        return pf_;
    }

};

}

#endif