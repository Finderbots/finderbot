#include <finderbot/path_execution.h>

#define PI 3.14159265

// Applies command velocities until you are within a threshold of desired angle
void Executor::turnTheta(double goal_theta) {
    if (goal_theta > current_theta_) {
        // ROS_INFO("TURN LEFT");
        cmd_.angular.z = -0.03; // LEFT
    }
    else if (goal_theta < current_theta_) {
        // ROS_INFO("TURN RIGHT");
        cmd_.angular.z = 0.03; // RIGHT
    }
    command_velocities_pub_.publish(cmd_);
    
    return;
}

// Returns a command velocity to move forward distance in feet (negative for backwards)
void Executor::drive(int goal_row, int goal_col) {
    cmd_.linear.x = -0.03;
    // ROS_INFO("FORWARD");
    command_velocities_pub_.publish(cmd_);
    return;
}


// // Input is a radius to the goal that you consider close enough
// bool Executor::closeEnoughToGoal(int radius, int goal_row, int goal_col) {
//     return distance(goal_row, goal_col, current_row_, current_col_) < radius;
// }

bool Executor::thetaCloseEnough(double threshold, double goal_theta) {
    return fabs(goal_theta - current_theta_) < threshold;
}

void Executor::goToNextNodeInPath(size_t goal_row, size_t goal_col) {
    // getPose();
    double dx = goal_row - current_row_;
    double dy = goal_col - current_col_;
    double goal_theta = atan(dy/dx); // RAD->DEGREE CONVERSION

    // while (!closeEnoughToGoal(0.5, goal_row, goal_col)) {
    moving_to_point_ = true;
    ROS_INFO("Goal pos (%zd, %zd, %f)", goal_row, goal_col, goal_theta);
    ROS_INFO("Start pos (%zd, %zd, %f)", current_row_, current_col_, current_theta_);

    while ((current_row_ != goal_row) && (current_col_ != goal_col))
    {
        // getPose();

        cmd_.linear.x = 0;
        cmd_.angular.z = 0;

        // We rotate to aim straight at the next point in the path then go straight to it
        // Do we need to wait till you finish turning? Or does it queue commands?
        // Is there a way to have it queue instead of one at a time?
        if (!thetaCloseEnough(0.05, goal_theta)) 
        {
            turnTheta(goal_theta);
        }
        else
        {
            drive(goal_row, goal_col);
        }
        
        ros::spinOnce();

    }

    moving_to_point_ = false;
    cmd_.linear.x = 0;
    cmd_.linear.y = 0;
    command_velocities_pub_.publish(cmd_);

    return;
}

void Executor::pathExecution(std::vector<size_t>& path) {
    // ROS_INFO("EXECUTE (pathExecution)");
    size_t goal_row = 0;
    size_t goal_col = 0;

    size_t map_width = planner_->getMapPtr()->info.width;

    ROS_INFO("PATH TO POINT (%zd, %zd)", map_utils::rowFromOffset(path.front(), map_width), map_utils::rowFromOffset(path.front(), map_width))
    int max_path_length = 5;
    int count = 0;

    while (!path.empty() && ros::ok()) {
        goal_row = map_utils::rowFromOffset(path.back(), map_width);
        goal_col = map_utils::colFromOffset(path.back(), map_width);
        path.pop_back();

        goToNextNodeInPath(goal_row, goal_col);
        if (count == max_path_length) 
        {   
            ROS_INFO("Get New Path");
            return; 
        }

        count++;
        ros::spinOnce();
    }

    ROS_INFO("Get New Path");
    return;
}

void Executor::handleGlobalMap(const nav_msgs::OccupancyGrid map)
{
    // ROS_INFO("HANDLING MAP");
    if (moving_to_point_) return;
    if (!map_initialized_)
    {
        // ROS_INFO("INITIALIZING MAP");
        map_initialized_ = true;
        planner_ = new Planner(map);

        // x_init_ = transform.getOrigin().x();
        // y_init_ = transform.getOrigin().y(); 

        // ROS_INFO("init = (%f, %f)", x_init_, y_init_);      
        init_map_x_ = map.info.height/2;
        init_map_y_ = map.info.width/2;
              
        // return;
    }
    // for (size_t i = 0; i < map.data.size(); i++)
    // {
    //     std::coutca << (int)map.data[i] << std::endl;
    // }

    planner_->updateMap(map);

    return;
}


void Executor::handlePose(const finderbot::Pose new_pose)
{
    if (!map_initialized_) 
    {
        // ROS_INFO("POSE BUT NO MAP");
        return; 
    }

    if (!pose_initialized_) 
    {
        ROS_INFO("FIRST POSE");
        x_init_ = new_pose.x;
        y_init_ = new_pose.y;
        pose_initialized_ = true;

    }

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
    command_velocities_pub_ = nh.advertise<geometry_msgs::Twist>("finderbot_cmd_vel", 1, true);
}