#ifndef RAY_CASTER_H
#define RAY_CASTER_H

#include <math.h>
#include <cmath>
#include <map>
#include <vector>

#include <angles/angles.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>

namespace ray_caster
{

typedef std::map<double, std::vector<size_t> > RayLookup;

class RayCaster
{
    RayLookup raycast_lookup_;
    int
     occupied_threshold_;
    size_t ncol_;
    size_t nrow_;
    RayLookup::const_iterator angleLookup(const double angle, const double tolerance);
public:
    RayCaster(const int
     occupied_threshold = 60);

    void laserScanCast(const nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& scan);

    const std::vector<size_t>& getRayCastToMapBorder(const double angle, const size_t nrow,
                                    const size_t ncol, const double tolerance = 0);
    size_t lookupSize() const
    {
        return raycast_lookup_.size();
    }

};

/* Return the row number from offset for a row-major array
 */
inline size_t rowFromOffset(const size_t offset, const size_t ncol)
{
    return offset / ncol;
}

/* Return the column number from offset for a row-major array
 */
inline size_t colFromOffset(const size_t offset, const size_t ncol)
{
    return offset % ncol;
}

/* Return the offset from row and column number for a row-major array
 */
inline size_t offsetFromRowCol(const size_t row, const size_t col, const size_t ncol)
{
    return (row * ncol) + col;
}

/* Return true if the point lies in the map
 */
inline bool pointInMap(const int row, const int col, const size_t nrow, const size_t ncol)
{
    return ((0 <= col) && ((size_t) col < ncol) && (0 <= row) && ((size_t) row < nrow));
}

/* Return the world coordinates of the map point at given index
 *
 * The map center is (0, 0).
 */
inline void indexToReal(const nav_msgs::OccupancyGrid& map, const size_t index, geometry_msgs::Point32& point)
{
    const double xcenter = (map.info.width / 2) * map.info.resolution;
    const double ycenter = (map.info.height / 2) * map.info.resolution;
    const size_t row = rowFromOffset(index, map.info.height);
    const size_t col = colFromOffset(index, map.info.width);
    const double xindex = col * map.info.resolution;
    const double yindex = row * map.info.resolution;
    point.x = xindex - xcenter;
    point.y = yindex - ycenter;
}

}

#endif