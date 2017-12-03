#include <ros/ros.h>
#include <ros/console.h>

#include <finderbot/map_utils.h>
#include <finderbot/PF_Input.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <random>
#include <string>

//job of SLAM is to make sure that position is correct
//by choosing pose that makes the raw lidar data correlate best with the map

struct Pose{
    double x;
    double y;
    double theta;
};

class SLAM
{
    Pose pose_;
    double std_dev_;

    size_t global_width_;
    size_t global_height_;

    int num_particles_;
    std::vector<Pose> particles_;

    ros::ServiceClient client_;
    gazebo_msgs::GetModelState model_state_;

    std::string world_frame_id_;
    std::string laser_frame_id_;

    tf::TransformBroadcaster br_;

public:
    //(num_particles, std_dev, world_frame_id, laser_frame_id, model_name
    SLAM(int num_particles, double std_dev, std::string world_frame_id, std::string laser_frame_id, std::string model_name)
        : 
        num_particles_(num_particles),
        std_dev_(std_dev),
        world_frame_id_(world_frame_id),
        laser_frame_id_(laser_frame_id)
    {
        particles_.reserve(num_particles_);
        ros::NodeHandle pnh("~");
        client_ = pnh.serviceClient<gazebo_msgs::GetModelState>("/gazebo_launch/get_model_state");
        model_state_.request.model_name = model_name;

        while(!client_.call(model_state_));

        geometry_msgs::Pose pose;
        pose = model_state_.response.pose;
        pose_.x = pose.position.x;
        pose_.y = pose.position.y;
        tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        pose_.theta = -1.5707;
        
        // ROS_INFO("init transform (%f, %f, %f)",pose_.x, pose_.y, pose_.theta);
        tf::Transform transform;

        q.setRPY(0, 0, pose_.theta);
        transform.setRotation(q);

        // tf::Vector3 position(pose.position.x, pose.position.y, 0);
        tf::Vector3 position(pose_.x, pose_.y, 0);
        transform.setOrigin(position);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                                world_frame_id_, laser_frame_id_));
        
    }


    void handlePFData(const finderbot::PF_Input pfInput)
    {
        // get deltas from IMU, get abs pose from sim
        geometry_msgs::Pose pose;
        if (client_.call(model_state_))
        {
             pose = model_state_.response.pose;
             pose_.x = pose.position.x;
             pose_.y = pose.position.y;
             tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
             pose_.theta = map_utils::convertQuatToAngle(q);
        }
        else ROS_INFO("boo gazebo");
        // ROS_INFO("abs_pose = (%f, %f, %f", pose_.x, pose_.y, pose_.theta);
        //fill particles vector by sampling from N(pose, std_dev) num_particles times
        std::default_random_engine generator;
        std::normal_distribution<double> x_distribution(pose_.x, std_dev_);
        std::normal_distribution<double> y_distribution(pose_.y, std_dev_);
        std::normal_distribution<double> theta_distribution(pose_.theta, std_dev_);

        particles_.clear();
        for (int i = 0; i < num_particles_; i++)
        {
            Pose particle;

            particle.x = x_distribution(generator);
            particle.y = y_distribution(generator);
            particle.theta = theta_distribution(generator);

            particles_.push_back(particle);
        }
        
        //loop through particles
        Pose max_pose;
        double max_score = 0.0;
        double score;
        for (size_t i = 0; i < particles_.size(); i++)
        {
            //loop through scans
            for (size_t j = 0; j < pfInput.scan_ranges.size(); j++)
            {
                //cast scan to global map position, from particles pose
                double world_theta = particles_[i].theta + pfInput.scan_angles[j];
                double world_x = particles_[i].x + pfInput.scan_ranges[j]*std::sin(world_theta);
                double world_y = particles_[i].y + pfInput.scan_ranges[j]*std::cos(world_theta);

                //get laser_global idx
                size_t map_x = (size_t) world_x / pfInput.map_resolution;
                size_t map_y = (size_t) world_y / pfInput.map_resolution;
                
                size_t idx = map_utils::getOffsetRowCol(map_x, map_y, pfInput.map_width);
                //if particlesrticle and scan yield a valid point then add to score
                if (map_utils::pointInMap(map_x, map_y, pfInput.map_height, pfInput.map_width) 
                        && pfInput.log_odds[idx] != 0)
                {
                    score += pfInput.log_odds[idx];   
                }
            }

            if (score > max_score)
            {
                max_score = score;
                max_pose = particles_[i];
            }
        }
        
        // ROS_INFO("pf_pose = (%f, %f, %f", max_pose.x, max_pose.y, max_pose.theta);
        //select particle with highest pose, update pose
        pose_ = max_pose;

        //publish transform with pose data
        tf::Transform transform;
        
        // tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf::Quaternion q;
        q.setRPY(0, 0, pose_.theta);
        transform.setRotation(q);

        // tf::Vector3 position(pose.position.x, pose.position.y, 0);
        tf::Vector3 position(pose_.x, pose_.y, 0);
        transform.setOrigin(position);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                                world_frame_id_, laser_frame_id_));
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_frame_transformer");
    ros::NodeHandle nh;

    std::string world_frame_id;
    std::string laser_frame_id;
    std::string model_name;
    int num_particles;
    double std_dev;

    nh.param<std::string>("world_frame_id", world_frame_id, "world");
    nh.param<std::string>("laser_frame_id", laser_frame_id, "laser_frame");
    nh.param<std::string>("model_name", model_name, "Finderbot_Lidar");
    nh.param<int>("pf_num_particles", num_particles, 1000);
    nh.param<double>("pf_std_dev", std_dev, 0.1);
    
    while ()
    SLAM slamma_jamma(num_particles, std_dev, world_frame_id, laser_frame_id, model_name);

    ros::Subscriber pf_handler = nh.subscribe<finderbot::PF_Input>("SLAM_pf", 1, &SLAM::handlePFData, &slamma_jamma);
    // ros::Subscriber laser_handler
   
    ros::spin(); 

}