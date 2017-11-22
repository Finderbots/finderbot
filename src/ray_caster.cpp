#include <finderbot/ray_caster.h>

namespace ray_caster
{

    RayCaster::RayCaster(const int occupied_threshold) :
        occupied_threshold_(occupied_threshold){}

    inline bool pointOccupied(const nav_msgs::OccupancyGrid& map, const int idx, const int occupied_threshold)
    {
        return (map.data[idx] > occupied_threshold) || (map.data[idx] == -1);
    }

    //TODO: Nothing uses this! get rid of it
    void RayCaster::laserScanCast(const nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& scan)
    {
        scan.ranges.clear(); //why???? I think we would want to use this
        for (double angle = scan.angle_min; angle <= scan.angle_max; angle += scan.angle_increment){

            const size_t pixel_range = lround(scan.range_max / map.info.resolution) + 1;
            const std::vector<size_t>& ray_indices = getRayCastToMapBorder(
                                                        angle,
                                                        map.info.height,
                                                        map.info.width,
                                                        scan.angle_increment/2);
            const size_t max_pixel_range = std::min(ray_indices.size(), pixel_range);
            geometry_msgs::Point32 pt;
            indexToReal(map, ray_indices.back(), pt); //pt is RW coords of end of ray
            //range is min of scan.range_max and distance to edge of map
            double range  = std::min(0.99*scan.range_max, (double) std::sqrt(pt.x*pt.x + pt.y*pt.y));
            for (size_t i = 0; i < max_pixel_range; i++)
            {
                const size_t idx = ray_indices[i];
                if(pointOccupied(map, idx, occupied_threshold_))
                {
                    indexToReal(map, idx, pt);
                    range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
                    break;
                }
            }
            scan.ranges.push_back(range);
            
        }
    }

    //function returns vector of pixel indices from map center to border of map given an angle
    //TODO adjust x0 and y0 so that starting position is wherever position is.
    //Assume that the orientation is forward or use some transform so it's equivalent
    const std::vector<size_t>& RayCaster::getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol, const double tolerance)
    {
        if (nrow != nrow_ || ncol != ncol_)
        {
            raycast_lookup_.clear();
            nrow_ = nrow;
            ncol_ = ncol;
        }

        RayLookup::const_iterator ray = angleLookup(angle, tolerance);
        if (ray != raycast_lookup_.end())
        {
            return ray->second;
        } //ray is already cached

        std::vector<size_t> pts;

        //distance from one corner of map to another
        const double r = std::sqrt((double) nrow * nrow + ncol*ncol);

        //assumption that origin is at map center
        int x0 = nrow / 2;
        int y0 = ncol / 2;

        int x1 = (int) round(x0 + r*std::sin(angle));
        int y1 = (int) round(y0 + r*std::cos(angle));

        int dx = x1 - x0;
        int dy = y1 - y0;

        bool steep (std::abs(dy) >= std::abs(dx));

        if (steep)
        {
            std::swap(x0,y0);
            std::swap(x1,y1);

            dx = x1 - x0;
            dy = y1 - y0;
        }

        int xstep = 1;
        if (dx < 0)
        {
            xstep = -1;
            dx = -dx;
        }

        int ystep = 1;
        if (dy < 0){
            ystep = -1;
            dy = -dy;
        }

        int e = 2*dy - dx;
        int y = y0;
        int xDraw, yDraw;

        for (int x = x0; x != x1; x += xstep)
        {
            if (steep){
                xDraw = y;
                yDraw = x;
            }
            else{
                xDraw = x;
                yDraw = y;
            }

            if (pointInMap(yDraw, xDraw, nrow, ncol))
            {
                pts.push_back(offsetFromRowCol(yDraw, xDraw, ncol));
            }
            else
            {
                raycast_lookup_[angle] = pts;
                return raycast_lookup_[angle];
            }

            if (e > 0)
            {
                e += 2*(dy - dx);
                y += ystep;
            }
            else
            {
                e += 2*dy;
            }
        }


    }

    //TODO:: figure out whether this expects angles to be -Pi -PI or 0 to 2PI 
    //and make sure that LaserScan from Hokuyo is consistent
    RayLookup::const_iterator RayCaster::angleLookup(const double angle, const double tolerance)
    {
        if (tolerance == 0)
        {
            return raycast_lookup_.find(angle);
        }

        //dangle lower / upper
        double upper_dist, lower_dist;
        //upper_bound points to first element with angle greater than angle
        RayLookup::const_iterator upper_bound = raycast_lookup_.upper_bound(angle);
        if (upper_bound == raycast_lookup_.begin())
        {
            //angle < first angle in cache
            if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
            {
                return upper_bound;
            }
            return raycast_lookup_.end();
        }

        else if (upper_bound == raycast_lookup_.end())
        {
            //angle greater than largest angle in cache
            upper_dist = raycast_lookup_.begin()->first - angle + 2*M_PI;
            upper_bound--;
            lower_dist = upper_bound->first - angle; 
            if (lower_dist < upper_dist)
            {
                if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
                {
                    return upper_bound;
                }
                return raycast_lookup_.end();
            }
            else
            {
                if (std::abs(angles::shortest_angular_distance(angle, raycast_lookup_.begin()->first)) <= tolerance)
                {
                    return upper_bound;
                }
                return raycast_lookup_.end();
            }
        }
        else
        {
            upper_dist = upper_bound->first - angle;
            RayLookup::const_iterator lower_bound = upper_bound;
            lower_bound--;
            lower_dist = angle-lower_bound->first;

            if (lower_dist < upper_dist)
            {
                if (std::abs(angles::shortest_angular_distance(angle, lower_bound->first)) <= tolerance)
                {
                    return lower_bound;
                }
                return raycast_lookup_.end();
            }
            else
            {
                if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
                {
                    return upper_bound;
                }
                return raycast_lookup_.end();
            }
        }
    }   
}