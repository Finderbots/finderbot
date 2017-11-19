#include <finderbot/global_map_builder.h>

namespace global_mapping
{
    const double default_p_occupied_laser = 0.9;
    const double default_p_occupied_no_laser = 0.3;
    const double default_max_log_odds = 100;
    const double default_belief_threshold = 20;

    GlobalMapBuilder::GlobalMapBuilder(int width, int height, double resolution, std::string laser_frame_id)
        :
        p_occupied_laser_(default_p_occupied_laser),
        p_occupied_no_laser_(default_p_occupied_no_laser),
        max_log_odds_(default_max_log_odds),
        belief_threshold_(default_belief_threshold)
    {
        map_.info.width = width;
        map_.info.height = height;
        map_.info.resolution = resolution;
        map_.info.origin.position.x = resolution * static_cast<double>(width) / 2;
        map_.info.origin.position.y = resolution * static_cast<double>(height) / 2;
        map_.info.origin.orientation.w = 1.0;


        map_.data.assign(width * height, -1);
        log_odds_map_.assign(width*height, 0);

        laser_frame_id_ = laser_frame_id;
        world_frame_id_ = "world";

        ros::NodeHandle private_nh("~");
        private_nh.getParam("p_occupied_laser", p_occupied_laser_);            
        private_nh.getParam("p_occupied_when_no_laser", p_occupied_no_laser_);
        private_nh.getParam("max_log_odds", max_log_odds_);
        private_nh.getParam("belief_threshold", belief_threshold_);
    }

    void GlobalMapBuilder::updatePosition()
    {
        if (!transform_initialized)
        {
            tf::StampedTransform transform;
            try
            {
                tf_listener_.lookupTransform(laser_frame_id_,world_frame_id_, 
                                ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }

            x_init_ = transform.getOrigin().x();
            y_init_ = transform.getOrigin().y();

            transform_initialized = true;
            //robot initialized in center of map
            init_map_x_ = map_.info.height/2;
            init_map_y_ = map_.info.width/2;
        }

        tf::StampedTransform new_transform;
        try
        {
            tf_listener_.lookupTransform(laser_frame_id_,world_frame_id_, 
                            ros::Time(0), new_transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        const double dx = new_transform.getOrigin().x() - x_init_;
        const double dy = new_transform.getOrigin().y() - y_init_;

        map_x_ = (dx / map_.info.resolution) + init_map_x_;
        map_y_ = (dy / map_.info.resolution) + init_map_y_;

    }

    void GlobalMapBuilder::updateProbOccupied(bool occupied,  size_t idx)
    {
        double p;
        if (occupied) p  = p_occupied_laser_;
        else p = p_occupied_no_laser_;
        log_odds_map_[idx] += std::log(p / (1-p));

        if (log_odds_map_[idx] > max_log_odds_) log_odds_map_[idx] = max_log_odds_;
        else if (log_odds_map_[idx] < -max_log_odds_) log_odds_map_[idx] = -max_log_odds_;
    
        if (log_odds_map_[idx] >= belief_threshold_) map_.data[idx] = 100;
        else map_.data[idx] = 0;

    }

    void GlobalMapBuilder::addLocalMap(const nav_msgs::OccupancyGrid& local_map)
    {
        updatePosition();

        assert(map_.info.resolution == local_map.info.resolution);

        size_t local_ncol = local_map.info.width;
        size_t local_nrow = local_map.info.height;

        size_t global_start_row = map_x_ - local_nrow/2;
        size_t global_start_col = map_y_ - local_ncol/2;

        for (size_t i = 0; i < local_nrow; i++)
        {
            for (size_t j = 0; j < local_ncol; j++)
            {
                size_t local_idx = getOffsetRowCol(i, j, local_ncol);
                size_t global_idx = getOffsetRowCol(global_start_col+i, 
                                                    global_start_row+j, 
                                                    map_.info.width);


                if(local_map.data[local_idx] == 100)
                {
                    updateProbOccupied(true, global_idx);
                    // map_.data[global_idx] = local_map.data[local_idx];
                }

                else if (local_map.data[local_idx] == 0)
                {
                    updateProbOccupied(false, global_idx);
                }
            }   
        }

    }   

        
}