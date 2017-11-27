#include <finderbot/global_map_builder.h>

namespace global_mapping
{
    const double default_p_occupied_laser = 0.9;
    const double default_p_occupied_no_laser = 0.3;
    const double default_max_log_odds = 100;
    const double default_belief_threshold = 20;

    double convertQuatToAngle(const tf::Quaternion& q)
    {
        if(std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5){
            tf::Vector3 axis = q.getAxis();
            // ROS_WARN("Laser frame rotation is not around the z-axis (axis = [%f, %f, %f], just pretending it is",
                // axis.x(), axis.y(), axis.z());
        }

        return 2*std::atan2(q.z(), q.w());
    }

    GlobalMapBuilder::GlobalMapBuilder(int global_width, int global_height, 
                                    int local_width, int local_height, 
                                    double resolution, std::string laser_frame_id)
        :
        angle_resolution_(M_PI/720),
        p_occupied_laser_(default_p_occupied_laser),
        p_occupied_no_laser_(default_p_occupied_no_laser),
        max_log_odds_(default_max_log_odds),
        belief_threshold_(default_belief_threshold)
    {

        global_map_.info.width = global_width;
        global_map_.info.height = global_height;
        global_map_.info.resolution = resolution;
        global_map_.info.origin.position.x = resolution * static_cast<double>(global_height) / 2;
        global_map_.info.origin.position.y = resolution * static_cast<double>(global_width) / 2;
        global_map_.info.origin.orientation.w = 1.0;

        global_map_.data.assign(global_width * global_height, -1);
        log_odds_map_.assign(global_width*global_height, 0);

        local_map_.info.width = local_width;
        local_map_.info.height = local_height;
        local_map_.info.resolution = resolution;

        local_map_.data.assign(local_height * local_height, -1);

        laser_frame_id_ = laser_frame_id;
        world_frame_id_ = "world";

        ros::NodeHandle private_nh("~");
        private_nh.getParam("p_occupied_laser", p_occupied_laser_);            
        private_nh.getParam("p_occupied_when_no_laser", p_occupied_no_laser_);
        private_nh.getParam("max_log_odds", max_log_odds_);
        private_nh.getParam("belief_threshold", belief_threshold_);

        const double angle_start = -M_PI;
        const double angle_end = angle_start + 2 * M_PI - 1e-6;
        for (double a = angle_start; a <= angle_end; a += angle_resolution_)
        {
            ray_caster_.getRayCastToMapBorder(a, local_height, local_width, 0.9 * angle_resolution_);
        }

        // perimeter of the local map is the max number of obstacles that can be detected.
        // pf_.probs.reserve(global_height * global_width);
        pf_.map_width = global_width;
        pf_.map_height = global_height;
        pf_.map_resolution = resolution;
        pf_.log_odds.reserve(2*global_height + 2*global_width);
        pf_.log_odds.assign(global_width*global_height, 0);

    }

    void GlobalMapBuilder::buildMapFromScan(const sensor_msgs::LaserScan& scan)
    {
        updatePosition();

        pf_.scan_ranges.clear();
        pf_.scan_angles.clear();

        createNewLocalMap(scan);

        addLocalMapToGlobal();
    }

    bool GlobalMapBuilder::castRayToObstacle(double angle, double range, std::vector<size_t>& raycast)
    {
        if (range < 1e-10)
        {
            ROS_WARN("TOO CLOSE");
            raycast.clear();
            return false;
        }
        const std::vector<size_t>& ray_to_border = 
                    ray_caster_.getRayCastToMapBorder(angle, local_map_.info.height,
                                         local_map_.info.width, 1.1*angle_resolution_);
        const size_t pixel_range = lround(range * std::max(fabs(std::cos(angle)), fabs(std::sin(angle))) / local_map_.info.resolution);
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


    void GlobalMapBuilder::updatePosition()
    {
        if (!transform_initialized)
        {
            ROS_INFO("transform uninitialized");
            tf::StampedTransform transform;
            try
            {
                //gazebo takes a while to startup so dont wait too long for a transform
                // tf_listener_.waitForTransform(world_frame_id_, laser_frame_id_, ros::Time(0), ros::Duration(1000.0));
                tf_listener_.lookupTransform(world_frame_id_, laser_frame_id_,
                                ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("initial woopsie %s", ex.what());
                return;
            }

            x_init_ = transform.getOrigin().x();
            y_init_ = transform.getOrigin().y();


            transform_initialized = true;
            //robot initialized in center of map
            init_map_x_ = global_map_.info.height/2;
            init_map_y_ = global_map_.info.width/2;
        }

        tf::StampedTransform new_transform;
        try
        {
            tf_listener_.lookupTransform(world_frame_id_, laser_frame_id_,
                            ros::Time(0), new_transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("mapping woopsie %s", ex.what());
            return;
        }

        const double dx = new_transform.getOrigin().x() - x_init_;
        const double dy = new_transform.getOrigin().y() - y_init_;

        map_x_ = (dx / global_map_.info.resolution) + init_map_x_;
        map_y_ = (dy / global_map_.info.resolution) + init_map_y_;

        theta_ = convertQuatToAngle(new_transform.getRotation()) + M_PI/2;
    }

    void GlobalMapBuilder::updateLocalOccupancy(bool occupied, size_t idx)
    {
        if (idx >= local_map_.data.size()) 
        {
            ROS_INFO("idx = %zd, total size = %zd", idx, local_map_.data.size());
            return;
        }

        if (occupied) local_map_.data[idx] = 100;
        else local_map_.data[idx] = 0;
    }

    void GlobalMapBuilder::vec_updateLocalOccupancy(bool occupied, std::vector<size_t>& ray)
    {
        for (size_t i = 0; i < ray.size(); i++)
        {
            updateLocalOccupancy(occupied, ray[i]);
        }
    }

    void GlobalMapBuilder::createNewLocalMap(const sensor_msgs::LaserScan& scan)
    {
        local_map_.data.assign(local_map_.info.width* local_map_.info.height, -1);

        const int ncol = local_map_.info.width;
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment + theta_);
            pf_.scan_ranges.push_back(scan.ranges[i]);
            pf_.scan_angles.push_back(scan.angle_min + i*scan.angle_increment);

            std::vector<size_t> pts;
            // ROS_INFO("range = %f", scan.ranges[i]);
            const bool obstacle_in_map = castRayToObstacle(angle, scan.ranges[i], pts);
            if (pts.empty())
            {
                continue;
            }
            if (obstacle_in_map)
            {
                // The last point is the point with obstacle.
                updateLocalOccupancy(true, pts.back());

                // ROS_INFO("occupancy val at (%zd,%zd) = %d", ray_caster::rowFromOffset(pts.back(), ncol),
                //                                     ray_caster::colFromOffset(pts.back(), ncol), 
                //                                     map_.data[pts.back()]);

                //this data is used to cast rays to global map in pf
                
                pts.pop_back();
            }
            // The remaining points are in free space.
            vec_updateLocalOccupancy(false, pts);
        }
    }

    void GlobalMapBuilder::updateProbOccupied(bool occupied,  size_t idx)
    {
        double p;
        if (occupied) p  = p_occupied_laser_;
        else p = p_occupied_no_laser_;
        log_odds_map_[idx] += std::log(p / (1-p));

        if (log_odds_map_[idx] > max_log_odds_) log_odds_map_[idx] = max_log_odds_;
        else if (log_odds_map_[idx] < -max_log_odds_) log_odds_map_[idx] = -max_log_odds_;
    
        if (log_odds_map_[idx] >= belief_threshold_) global_map_.data[idx] = 100;
        else if (log_odds_map_[idx] < -belief_threshold_) global_map_.data[idx] = 0;
        else global_map_.data[idx] = static_cast<int8_t>((1 - 1/ (1 + std::exp(log_odds_map_[idx])))*100);

    }

    void GlobalMapBuilder::addLocalMapToGlobal()
    {
        assert(global_map_.info.resolution == local_map_.info.resolution);
        updatePosition();

        size_t local_ncol = local_map_.info.width;
        size_t local_nrow = local_map_.info.height;

        size_t global_start_row = map_x_ - local_nrow/2;
        size_t global_start_col = map_y_ - local_ncol/2;

        for (size_t i = 0; i < local_nrow; i++)
        {
            for (size_t j = 0; j < local_ncol; j++)
            {
                size_t local_idx = getOffsetRowCol(i, j, local_ncol);
                size_t global_idx = getOffsetRowCol(global_start_row+i, 
                                                    global_start_col+j, 
                                                    global_map_.info.width);

                if(local_map_.data[local_idx] == 100)
                {
                    updateProbOccupied(true, global_idx);
                }

                else if (local_map_.data[local_idx] == 0)
                {
                    updateProbOccupied(false, global_idx);
                }

                pf_.log_odds[global_idx] = log_odds_map_[global_idx];
            }   
        }
    }   
}