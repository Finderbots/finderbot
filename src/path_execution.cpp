#include <finderbot/path_execution.h>

#define PI 3.14159265
#define DIST_THRESH 2
// Applies command velocities until you are within a threshold of desired angle

inline double angle_diff(double leftAngle, double rightAngle)
{
    double diff = leftAngle - rightAngle;
    // ROS_INFO("init diff = %f", diff);
    if(fabs(diff) > PI)
    {
        diff -= (diff > 0) ? PI*2 : PI*-2;
    }
    return diff;
}

inline float wrap_to_pi(float angle)
{
    if(angle < -M_PI)
    {
        for(; angle < -M_PI; angle += 2.0*M_PI);
    }
    else if(angle > M_PI)
    {
        for(; angle > M_PI; angle -= 2.0*M_PI);
    }

    return angle;
}

bool Executor::turnTheta(double error) {

    if (error > 0) {
        // ROS_INFO("TURN LEFT");
        cmd_.angular.z = 0.03; // LEFT
    }
    else if (error < 0) {
        // ROS_INFO("TURN RIGHT");
        cmd_.angular.z = -0.03; // RIGHT
    }
    command_velocities_pub_.publish(cmd_);

    if (error > 0) return true;
    else return false;
}

// Returns a command velocity to move forward distance in feet (negative for backwards)
void Executor::drive() {
    cmd_.linear.x = 0.05;
    // ROS_INFO("FORWARD");
    command_velocities_pub_.publish(cmd_);
    return;
}


// Input is a radius to the goal that you consider close enough
bool Executor::closeEnoughToGoal(double radius, int goal_row, int goal_col) {

    double dist = map_utils::distance(goal_row, goal_col, current_row_, current_col_); 
    return dist < radius;
}

// bool Executor::thetaCloseEnough(double threshold, double goal_theta) {
//     return fabs(goal_theta - current_theta_) < threshold;
// }

void Executor::goToNextNodeInPath(size_t goal_row, size_t goal_col) {
    
    double dx = (int)goal_row - (int)current_row_;
    double dy = (int)goal_col - (int)current_col_;

    double goal_theta = std::atan2(dy, dx);
    // while (!closeEnoughToGoal(0.5, goal_row, goal_col)) {
    moving_to_point_ = true;
    // ROS_INFO("Goal pos (%zd, %zd, %f)", goal_row, goal_col, goal_theta);
    // ROS_INFO("start pos (%zd, %zd, %f)", current_row_, current_col_, current_theta_);

    size_t print_count = 0;
    bool left;
    while (!closeEnoughToGoal(10, goal_row, goal_col))
    {
        dx = (int)goal_row - (int)current_row_;
        dy = (int)goal_col - (int)current_col_;

        goal_theta = std::atan2(dy, dx);
        double dist = map_utils::distance(goal_row, goal_col, current_row_, current_col_); 

        double angle_error = angle_diff(goal_theta, current_theta_);
        
      
        // ROS_INFO("GOAL (%zd,%zd, %f), CURR(%zd, %zd, %f) , error = %f, dist = %f", goal_row, goal_col, goal_theta, current_row_, current_col_,current_theta_, angle_error, dist);
        
        cmd_.linear.x = 0;
        cmd_.angular.z = 0;

        // We rotate to aim straight at the next point in the path then go straight to it
        // Do we need to wait till you finish turning? Or does it queue commands?
        // Is there a way to have it queue instead of one at a time?
        if (fabs(angle_error) > 0.25) 
        {
            left = turnTheta(angle_error);
        }
        else
        {
            drive();
        }


        // if (left) ROS_INFO("TURN LEFT theta = %f, goal = %f, error = %f", current_theta_, goal_theta, angle_error);
        // else ROS_INFO("TURN RIGHT theta = %f, goal = %f, error = %f", current_theta_, goal_theta, angle_error);
        // ROS_INFO("GOAL (%zd,%zd, %f), CURR(%zd, %zd, %f) , error = %f, dist = %f", goal_row, goal_col, goal_theta, current_row_, current_col_,current_theta_, angle_error, dist);

        ros::Duration(0.2).sleep();

        ROS_INFO("execute next Node spin");
        //should only update Pose
        ros::spinOnce();

    }

    // if (current_row_ != goal_row || current_col_ != goal_col)
    // {
    //     ROS_ERROR("WTF your'e at (%zd, %zd), should be at *(%zd,%zd)", current_row_, current_col_, goal_row, goal_col);
    // }

    moving_to_point_ = false;

    //TODO test only stopping at end of path??
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

    ROS_INFO("PATH TO POINT (%zd, %zd)", map_utils::rowFromOffset(path.front(), map_width), map_utils::colFromOffset(path.front(), map_width));
    
    int max_path_length = 25;
    int count = 0;

    while (!path.empty() && ros::ok()) {
        goal_row = map_utils::rowFromOffset(path.back(), map_width);
        goal_col = map_utils::colFromOffset(path.back(), map_width);
        path.pop_back();

        goToNextNodeInPath(goal_row, goal_col);
        if (count >= max_path_length) 
        {
            ROS_INFO("Get New Path");
            return; 
        }

        count++;
        //update Map
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

        init_map_x_ = map.info.height/2;
        init_map_y_ = map.info.width/2;
              
    }

    ROS_INFO("UPDATE MAP");
    

    planner_->updateMap(map);

    return;
}


void Executor::handlePose(const finderbot::Pose new_pose)
{
    // ROS_INFO("handlePOSe");
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

    double map_resolution = planner_->getMapPtr()->info.resolution;

    const double dx = new_pose.x - x_init_;
    const double dy = new_pose.y - y_init_;

    current_row_ = (dx / map_resolution) + init_map_x_;
    current_col_ = (dy / map_resolution) + init_map_y_;

    current_theta_ = wrap_to_pi(new_pose.theta + M_PI/2);
    // ROS_INFO("NEW POSE = (%zd, %zd, %f)", current_row_, current_col_, current_theta_);
    planner_->setPose(current_row_, current_col_);
}

Executor::Executor(std::string world_frame, std::string local_frame)
    : world_frame_id_(world_frame), local_frame_id_(local_frame)
{
    command_velocities_pub_ = nh.advertise<geometry_msgs::Twist>("finderbot_cmd_vel", 1, true);
    //client_ = nh.serviceClient<finderbot::UWBScan>("UWB_scan");
}
