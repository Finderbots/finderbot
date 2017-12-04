#include <finderbot/path_execution.h>

#define PI 3.14159265

// Applies command velocities until you are within a threshold of desired angle
void Executor::turnTheta(double goal_theta) {
    if (goal_theta > current_theta_) {
        cmd_.angular.z = -0.05; // LEFT
    }
    else if (goal_theta < current_theta_) {
        cmd_.angular.z = 0.05; // RIGHT
    }
    command_velocities_pub_.publish(cmd_);
    
    return;
}

// Returns a command velocity to move forward distance in feet (negative for backwards)
void Executor::drive(int goal_row, int goal_col) {
    cmd_.linear.x = -0.03;
    command_velocities_pub_.publish(cmd_);
    return;
}

// // Input is a radius to the goal that you consider close enough
// bool Executor::closeEnoughToGoal(int radius, int goal_row, int goal_col) {
//     return distance(goal_row, goal_col, current_row_, current_col_) < radius;
// }

bool Executor::thetaCloseEnough(double threshold, double goal_theta) {
    return (goal_theta - current_theta_) < threshold;
}

// void Executor::getPose()
// {
//     // tf::StampedTransform transform;

//     // tf_listener_.lookupTransform(world_frame_id_, local_frame_id_, 
//     //                                 ros::Time(0), transform);

//     // const double dx = transform.getOrigin().x() - x_init_;
//     // const double dy = transform.getOrigin().y() - y_init_;

//     // current_row_ = (dx / map_resolution) + init_map_x_;
//     // current_col_ = (dy / map_resolution) + init_map_y_;

//     // current_theta_ = map_utils::convertQuatToAngle(transform.getRotation()) + M_PI/2;

//     // planner_->setPose(current_row_, current_col_);
// }   


void Executor::goToNextNodeInPath(size_t goal_row, size_t goal_col) {
    // getPose();
    double goal_theta = atan((goal_row-current_row_)/(goal_col-current_col_)) * (180 / PI); // RAD->DEGREE CONVERSION
    
    // while (!closeEnoughToGoal(0.5, goal_row, goal_col)) {
    while ((current_row_ != goal_row) && (current_col_ != goal_col))
    {
        // getPose();

        cmd_.linear.x = 0;
        cmd_.angular.z = 0;

        // We rotate to aim straight at the next point in the path then go straight to it
        // Do we need to wait till you finish turning? Or does it queue commands?
        // Is there a way to have it queue instead of one at a time?
        if (!thetaCloseEnough(0.1, goal_theta)) 
        {
            turnTheta(goal_theta);
        }
        else
        {
            drive(goal_row, goal_col);
        }
    }

    return;
}

void Executor::pathExecution(std::vector<size_t>& path) {

    int goal_row = 0;
    int goal_col = 0;
    while (!path.empty() && ros::ok()) {
        goal_row = map_utils::rowFromOffset(path.back(), global_width);
        goal_col = map_utils::colFromOffset(path.back(), global_width);
        path.pop_back();
        goToNextNodeInPath(goal_row, goal_col);
        ros::spinOnce();
    }
    return;
}

void Executor::handleGlobalMap(const nav_msgs::OccupancyGrid map)
{
    ROS_INFO("HANDLING MAP");

    if (!map_initialized_)
    {
        ROS_INFO("INITIALIZING MAP");
        map_initialized_ = true;
        planner_ = new Planner(map);

        // x_init_ = transform.getOrigin().x();
        // y_init_ = transform.getOrigin().y(); 

        // ROS_INFO("init = (%f, %f)", x_init_, y_init_);      
        // init_map_x_ = map.info.height/2;
        // init_map_y_ = map.info.width/2;
              
        return;
    }

    planner_->updateMap(map);
    return;
}


void Executor::handlePose(const finderbot::Pose new_pose)
{
    if (!map_initialized_) return;

    const double dx = new_pose.x - x_init_;
    const double dy = new_pose.y - y_init_;

    current_row_ = (dx / map_resolution) + init_map_x_;
    current_col_ = (dy / map_resolution) + init_map_y_;

    current_theta_ = new_pose.theta + M_PI/2;

    planner_->setPose(current_row_, current_col_);
}

Executor::Executor(std::string world_frame, std::string local_frame)
    : world_frame_id_(world_frame), local_frame_id_(local_frame)
{
    ros::NodeHandle nh;

    command_velocities_pub_ = nh.advertise<geometry_msgs::Twist>("command_velocities", 1, true);
}