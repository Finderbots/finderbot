#ifndef MAP_UTILS_H_
#define MAP_UTILS_H_

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <geometry_msgs/Point32.h>

namespace map_utils{
// Pass 2 sets of row and column
inline int distance(int row1, int col1, int row2, int col2) {
    // Euclidean distance
    return sqrt(pow(row1-row2,2) + pow(col1-col2,2));
}

/* Return true if the point lies in the map */
inline bool pointInMap(const int row, const int col, const nav_msgs::OccupancyGrid & global_map)
{
    return ((0 <= col) && ((size_t) col < global_map.info.width) && (0 <= row) && ((size_t) row < global_map.info.height));
}

inline bool pointInMap(const int row, const int col, const size_t nrow, const size_t ncol)
{
    return ((0 <= col) && ((size_t) col < ncol) && (0 <= row) && ((size_t) row < nrow));
}

//return index into map from row and col
inline size_t getOffsetRowCol(size_t row, size_t col, const nav_msgs::OccupancyGrid & global_map)
{
    return (row * global_map.info.width) + col;
}

inline size_t getOffsetRowCol(size_t row, size_t col, size_t ncol)
{
    return (row * ncol) + col;
}

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

inline double convertQuatToAngle(const tf::Quaternion& q)
{
    if(std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5){
        tf::Vector3 axis = q.getAxis();
        // ROS_WARN("Laser frame rotation is not around the z-axis (axis = [%f, %f, %f], just pretending it is",
            // axis.x(), axis.y(), axis.z());
    }

    return 2*std::atan2(q.z(), q.w());
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