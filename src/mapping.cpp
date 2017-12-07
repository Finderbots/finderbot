#include <math.h>
#include <cmath>
#include <finderbot/mapping.h>

//TODO :: clean this up so its more readable and take out unused calculations

namespace mapping
{
    const double default_p_occupied_laser = 0.9;
    const double default_p_occupied_no_laser = 0.3;
    const double default_log_odds = 100;
    const double default_max_log_odds_belief = 20;

    double convertQuatToAngle(const tf::Quaternion& q)
    {
        if(std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5){
            tf::Vector3 axis = q.getAxis();
            // ROS_WARN("Laser frame rotation is not around the z-axis (axis = [%f, %f, %f], just pretending it is",
                // axis.x(), axis.y(), axis.z());
        }

        return 2*std::atan2(q.z(), q.w());
    }



    MiniMapper::MiniMapper(int width, int height, double resolution, std::string local_frame_id) :
        angle_resolution_(M_PI/720),
        p_occupied_laser_(default_p_occupied_laser),
        p_occupied_no_laser_(default_p_occupied_no_laser),
        large_log_odds_(default_log_odds),
        max_log_odds_belief_(default_max_log_odds_belief)
        {
            map_.info.width = width;
            map_.info.height = height;
            map_.info.resolution = resolution;
            map_.info.origin.position.x = -static_cast<double>(width) / 2*resolution;
            map_.info.origin.position.y = -static_cast<double>(height) / 2*resolution;
            map_.info.origin.orientation.w = 1.0;
            map_.data.assign(width*height, -1);

            log_odds_.assign(width*height, 0);

            local_frame_id_ = local_frame_id;
            // map_frame_id_ = "map_frame_id";

            ros::NodeHandle private_nh("~");
            private_nh.getParam("angle_resolution", angle_resolution_);
            private_nh.getParam("p_occupied_laser", p_occupied_laser_);            
            private_nh.getParam("p_occupied_when_no_laser", p_occupied_no_laser_);
            private_nh.getParam("large_log_odds", large_log_odds_);
            private_nh.getParam("max_log_odds_for_belief", max_log_odds_belief_);
            
            const double angle_start = -M_PI;
            const double angle_end = angle_start + 2 * M_PI - 1e-6;
            for (double a = angle_start; a <= angle_end; a += angle_resolution_)
            {
                ray_caster_.getRayCastToMapBorder(a, height, width, 0.9 * angle_resolution_);
            }

        }
    void MiniMapper::buildMap(const sensor_msgs::LaserScan& scan)
    {
        if (!has_frame_id_)
        {
        // Wait for a parent.
            // ROS_INFO("enter no frame_id block");
         
            x_init_ = map_.info.height /2;
            y_init_ = map_.info.width /2;

            // map_x_ = lround(x_init_ / map_.info.resolution);
            // map_y_ = lround(y_init_ / map_.info.resolution);

        }

        // Map position relative to initialization.
     
        const double theta = 0;

        // Get the pixel displacement of the map.

        // Update the map
        const bool move = updateMap(scan, 0, 0, theta);
        // if (move)
        // {
        //     // ROS_INFO("Displacement: %ld, %ld pixels", map_dx, map_dy);
        // // Record the position only if the map moves.
        //     prev_map_x_ = xmap;
        //     prev_map_y_ = ymap;
        // }
        // Update the map frame, so that it's oriented like frame named "world_frame_id_".
        // tf::Transform map_transform;
        // map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // tf::Quaternion q;
        // q.setRPY(0, 0, -theta);
        // map_transform.setRotation(q);
        // tf_broadcaster_.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), local_frame_id_, map_frame_id_));
    }

    bool MiniMapper::castRayToObstacle(const nav_msgs::OccupancyGrid& map, double angle, double range, std::vector<size_t>& raycast)
    {
        //TODO actual minimum range
        if (range < 1e-10)
        {
            ROS_INFO("ya ray ain't shit");
            raycast.clear();
            return false;
        }

        const std::vector<size_t>& ray_to_border = ray_caster_.getRayCastToMapBorder(
                                                angle, map.info.height, map.info.width, 1.1*angle_resolution_);

        
        const size_t pixel_range = lround(range * std::max(fabs(std::cos(angle)), fabs(std::sin(angle))) / map.info.resolution);
        size_t raycast_size;
        if(pixel_range < 0)
        {
            ROS_WARN("Negative pixel range How??");
        }

        bool obstacle_in_map = std::abs(pixel_range) < ray_to_border.size();

        if (obstacle_in_map)
        {
            raycast_size = pixel_range;
        }
        else
        {
            raycast_size = ray_to_border.size();
        }
        
        raycast.clear();
        raycast.reserve(raycast_size);

        for (size_t i = 0; i < raycast_size; i++)
        {
            raycast.push_back(ray_to_border[i]);
        }
        return obstacle_in_map;
        
    }

    void MiniMapper::updateOccupancyVal(bool occupied, size_t idx, std::vector<int8_t>& occupancy, std::vector<double>& log_odds) const
    {
        if (idx >= occupancy.size()) return;
        if (occupancy.size() != log_odds.size()) return;

        double p;
        if (occupied) p = p_occupied_laser_;
        else p = p_occupied_no_laser_;

        log_odds[idx] += std::log(p / (1-p));
        if (log_odds[idx] < -large_log_odds_) log_odds[idx] = -large_log_odds_;
        else if (log_odds[idx] > large_log_odds_) log_odds[idx] = large_log_odds_;

        if (log_odds[idx] < max_log_odds_belief_) occupancy[idx] = 0;
        else  occupancy[idx] = 100;
        // else occupancy[idx] = static_cast<int8_t>(lround(1 - 1/ (1 + std::exp(log_odds[idx]))*100));
    }

    void fakeupdate(bool occupied, size_t idx, std::vector<int8_t>& occupancy)
    {
        if (idx >= occupancy.size()) 
        {
            ROS_WARN("IDX > OccupancyGrid");
            return;
        }

        if (occupied) occupancy[idx] = 100;
        else occupancy[idx] = 0;
    }

    inline void vec_fakeupdate(bool occupied, std::vector<size_t>& ray, std::vector<int8_t>& occupancy)
    {
        for (size_t i = 0; i < ray.size(); i++)
        {
            fakeupdate(occupied, ray[i], occupancy);
        }
    }

    bool MiniMapper::updateMap(const sensor_msgs::LaserScan& scan, long int dx, long int dy, double theta)
    {
        // ROS_INFO("UPDATE Map");
      

        // Update occupancy.
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment + theta);
            std::vector<size_t> pts;
            // ROS_INFO("range = %f", scan.ranges[i]);
            const bool obstacle_in_map = castRayToObstacle(map_, angle, scan.ranges[i], pts);
            if (pts.empty())
            {
                ROS_INFO("obstacle outside map");
                continue;
            }
            if (obstacle_in_map)
            {
                // The last point is the point with obstacle.
                fakeupdate(true, pts.back(), map_.data);

                // ROS_INFO("occupancy val at (%zd,%zd) = %d", ray_caster::rowFromOffset(pts.back(), ncol),
                //                                     ray_caster::colFromOffset(pts.back(), ncol), 
                //                                     map_.data[pts.back()]);
                pts.pop_back();
            }
            // The remaining points are in free space.
            vec_fakeupdate(false, pts, map_.data);
        }
        return false;
    }

}