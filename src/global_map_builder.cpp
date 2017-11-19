#include <finderbot/global_map_builder.h>

namespace global_mapping
{
    GlobalMapBuilder::GlobalMapBuilder(int width, int height, double resolution, std::string laser_frame_id)
    {
        ROS_INFO("HELLO");
        map_.info.width = width;
        map_.info.height = height;
        map_.info.resolution = resolution;
        map_.info.origin.position.x = resolution * static_cast<double>(width) / 2;
        map_.info.origin.position.y = resolution * static_cast<double>(height) / 2;

        map_.info.origin.orientation.w = 1.0;
        map_.data.assign(width * height, -1);

        laser_frame_id_ = laser_frame_id;
        world_frame_id_ = "world";

        //TODO add log_odds stuff
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
    void GlobalMapBuilder::addLocalMap(const nav_msgs::OccupancyGrid& local_map)
    {
        updatePosition();

        assert(map_.info.resolution == local_map.info.resolution);

        size_t local_ncol = local_map.info.width;
        size_t local_nrow = local_map.info.height;

        size_t global_start_row = map_x_ - local_nrow/2;
        size_t global_start_col = map_y_ - local_ncol/2;
        ROS_INFO("start_row = %zd", global_start_row);
        ROS_INFO("start_col = %zd", global_start_col);

        for (size_t i = 0; i < local_nrow; i++)
        {
            for (size_t j = 0; j < local_ncol; j++)
            {
                size_t local_idx = getOffsetRowCol(i, j, local_ncol);
                size_t global_idx = getOffsetRowCol(global_start_col+i, 
                                                    global_start_row+j, 
                                                    map_.info.width);
                if(local_map.data[local_idx] != -1)
                {
                    map_.data[global_idx] = local_map.data[local_idx];
                }
            }   
        }

    }   

        
}