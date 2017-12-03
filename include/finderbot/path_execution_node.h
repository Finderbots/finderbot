#include <finderbot/planner.h>
#include <finderbot/map_utils.h>
#include <finderbot/exploration.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <string>

static double map_resolution = 0.2;

class Executor
{
    int current_row_;
    int current_col_;

    double x_init_;
    double y_init_;

    size_t init_map_x_;
    size_t init_map_y_;

    double current_theta_;

    bool map_initialized_;

    Planner* planner_;

    geometry_msgs::Twist cmd_;
    std::string world_frame_id_;
    std::string local_frame_id_;

    tf::TransformListener tf_listener_;
    ros::Publisher command_velocities_pub_; 

    void turnTheta(double goal_theta);
    void drive(int goal_row, int goal_col);

    bool thetaCloseEnough(double threshold, double goal_theta);
    void goToNextNodeInPath(size_t goal_row, size_t goal_col);
    void getPose();


  public:
	Executor(std::string world_frame, std::string local_frame) 
        ;

    void pathExecution(std::vector<size_t>& path);
    void handleGlobalMap(const nav_msgs::OccupancyGrid);

    Planner& getPlanner() {return *planner_;}
    bool initialized() {return map_initialized_;}
	// ~Executor();
	
};