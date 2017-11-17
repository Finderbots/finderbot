#include <math.h>
#include <cmath>
#include <finderbot/mapping.h>

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
            ROS_WARN("Laser frame rotation is not around the z-axis (axis = [%f, %f, %f], just pretending it is",
                axis.x(), axis.y(), axis.z());
        }

        return 2*std::atan2(q.z(), q.w());
    }

    std::string getWorldFrame(const tf::Transformer& tf_transformer, const std::string& child)
    {
        std::string last_parent = child;
        std::string parent;
        bool has_parent = tf_transformer.getParent(child, ros::Time(0), parent);
        while (has_parent)
        { 
            last_parent = parent;
            has_parent = tf_transformer.getParent(parent, ros::Time(0), parent);
        }
        return last_parent;
    }



    template <typename T>
    void adjustMapForMovement(int fill, int dx, int dy, unsigned int ncol, std::vector<T>& map)
    {
        if (dx ==0 && dy ==0) return;

        const unsigned int nrow = map.size() / ncol;
        int row_start = 0;
        int row_end = nrow;
        int row_increment = 1;
        if (dy < 0)
        {
            row_start = nrow - 1;
            row_end = -1;
            row_increment = -1;
        }
        int col_start = 0;
        int col_steps = ncol;
        int col_increment = 1;
        if (dx < 0)
        {
            col_start = ncol - 1;
            col_steps = -ncol;
            col_increment = -1;
        }
        for (int new_row = row_start; new_row != row_end; new_row += row_increment)
        {
            const int new_idx_start = offsetRowCol(new_row, col_start, ncol); //idx of pt (new_row,col_start)
            const int row = new_row + dy;  // row in old map, can be outside old map
            int idx = offsetRowCol(row, col_start + dx, ncol); //idx of (new_row+dy, col_start+d)
            const int min_idx = std::max(0, offsetRowCol(row, 0, ncol));
            const int max_idx = std::min(static_cast<int>(map.size()) - 1, offsetRowCol(row, ncol - 1, ncol));
            const int new_idx_end = new_idx_start + col_steps;
            for (int new_idx = new_idx_start; new_idx != new_idx_end;)
            {
                if (min_idx <= idx && idx <= max_idx)
                {
                    map[new_idx] = map[idx];
                }
                else
                {
                    map[new_idx] = fill;
                }
                new_idx += col_increment;
                idx += col_increment;
            }
        }
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
            std::string parent;
            bool has_parent = tf_listener_.getParent(local_frame_id_, ros::Time(0), parent);  
            if (!has_parent)
            {
              ROS_INFO_STREAM("No worldframe");
              ROS_DEBUG_STREAM("Frame " << local_frame_id_ << " has no parent");
              return;
            }
            // ROS_INFO("parent is %s", parent.c_str());
            world_frame_id_ = getWorldFrame(tf_listener_, local_frame_id_);
            // ROS_INFO_STREAM("Found world frame " << world_frame_id_);
            // ROS_INFO("world_frame_id_ = %s", world_frame_id_.c_str());
            has_frame_id_ = true;

            // Initialize saved positions.
            tf::StampedTransform transform;
            try
            {
                // tf_listener_.waitForTransform(world_frame_id_, local_frame_id_,
                //   ros::Time(0), ros::Duration(1.0));
                tf_listener_.lookupTransform(world_frame_id_, local_frame_id_,
                  ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                has_frame_id_ = false;
            }
            x_init_ = transform.getOrigin().x();
            y_init_ = transform.getOrigin().y();
            prev_map_x_ = lround(x_init_ / map_.info.resolution);
            prev_map_y_ = lround(y_init_ / map_.info.resolution);

            // Send a map frame with identity transform.
            // tf::Transform map_transform;
            // map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            // map_transform.setRotation(tf::Quaternion(1, 0, 0, 0));
            // tf_broadcaster_.sendTransform(tf::StampedTransform(map_transform,
            // ros::Time::now(), local_frame_id_, map_frame_id_));
        }

        // Get the displacement.
        tf::StampedTransform new_tr;
        try
        {
            // tf_listener_.waitForTransform(world_frame_id_, local_frame_id_,
            //     ros::Time::now(), ros::Duration(0.2));
            tf_listener_.lookupTransform(world_frame_id_, local_frame_id_,
                ros::Time(0), new_tr);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // Map position relative to initialization.
        const double x = new_tr.getOrigin().x() - x_init_;
        const double y = new_tr.getOrigin().y() - y_init_;
        const double theta = convertQuatToAngle(new_tr.getRotation());

        // Get the pixel displacement of the map.
        const long int xmap = lround(x / map_.info.resolution);
        const long int ymap = lround(y / map_.info.resolution);
        const long int map_dx = xmap - prev_map_x_;
        const long int map_dy = ymap - prev_map_y_;

        // Update the map
        const bool move = updateMap(scan, map_dx, map_dy, theta);
        if (move)
        {
        //ROS_INFO("Displacement: %ld, %ld pixels", map_dx, map_dy);
        // Record the position only if the map moves.
            prev_map_x_ = xmap;
            prev_map_y_ = ymap;
        }
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
            ROS_INFO("YA RAY AINT SHIT");
            raycast.clear();
            return false;
        }

        const std::vector<size_t>& ray_to_border = ray_caster_.getRayCastToMapBorder(
                                                angle, map.info.height, map.info.width, 1.1*angle_resolution_);

        
        const size_t pixel_range = lround(range * std::max(fabs(std::cos(angle)), fabs(std::sin(angle))) / map.info.resolution);

        // ROS_INFO("angle = %f", angle);
        // ROS_INFO("resolution = %f", map.info.resolution);
        // ROS_INFO("pixel range = %zd", pixel_range);
        // ROS_INFO("range_to_border = %zd", ray_to_border.size());
        
        raycast.clear();
        if(std::abs(pixel_range) > ray_to_border.size())
        {
            for (size_t i = 0; i < ray_to_border.size(); i++)
            {
                raycast.push_back(ray_to_border[i]);
            }
            return false;
        }
        
        raycast.reserve(pixel_range);

        for (size_t i = 0; i < pixel_range; i++)
        {
            raycast.push_back(ray_to_border[i]);
        }
        return true;
        
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

        if (log_odds[idx] < -max_log_odds_belief_) occupancy[idx] = 0;
        else if (log_odds[idx] > max_log_odds_belief_) occupancy[idx] = 100;
        else occupancy[idx] = static_cast<int8_t>(lround(1 - 1/ (1 + std::exp(log_odds[idx]))*100));
    }


    bool MiniMapper::updateMap(const sensor_msgs::LaserScan& scan, long int dx, long int dy, double theta)
    {
        // ROS_INFO("UPDATE Map");
        const bool has_moved = (dx != 0 || dy != 0);
        const int ncol = map_.info.width;
        if (has_moved)
        {
        // Move the map and log_odds_.
            // ROS_INFO("Map moved");
            adjustMapForMovement(-1, dx, dy, ncol, map_.data);
            adjustMapForMovement(0, dx, dy, ncol, log_odds_);
        }

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
                updateOccupancyVal(true, pts.back(), map_.data, log_odds_);
                // ROS_INFO("occupancy val at (%zd,%zd) = %d", ray_caster::rowFromOffset(pts.back(), ncol),
                //                                     ray_caster::colFromOffset(pts.back(), ncol), 
                //                                     map_.data[pts.back()]);
                pts.pop_back();
            }
            // The remaining points are in free space.
            vec_updateOccupancy(false, pts, map_.data, log_odds_);
        }
        return has_moved;
    }

}