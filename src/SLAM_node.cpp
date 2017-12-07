#include <ros/ros.h>
#include <ros/console.h>

#include <finderbot/map_utils.h>
#include <finderbot/PF_Input.h>
#include <finderbot/Pose.h>
#include <finderbot/getImuPose.h>

#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <random>
#include <string>

//job of SLAM is to make sure that position is correct
//by choosing pose that makes the raw lidar data correlate best with the map

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

class SLAM
{
    finderbot::Pose pose_;
    finderbot::PF_Input pfInput;
    double std_dev_;

    size_t global_width_;
    size_t global_height_;

    int num_particles_;
    bool new_pf_data = false;
    bool simulated_env_;
    std::vector<finderbot::Pose> particles_;

    ros::ServiceClient gazebo_client_;
    ros::ServiceClient IMU_client_;

    gazebo_msgs::GetModelState model_state_;
    finderbot::getImuPose imu_pose_;

    ros::Publisher pose_publisher_;

public:
    //(num_particles, std_dev, world_frame_id, laser_frame_id, model_name
    SLAM(bool simulated_env, int num_particles, double std_dev, std::string model_name)
        : 
        simulated_env_(simulated_env),
        num_particles_(num_particles),
        std_dev_(std_dev)
    {
        particles_.reserve(num_particles_);
        ros::NodeHandle nh;

        pose_publisher_ = nh.advertise<finderbot::Pose>("finderbot_pose", 1, true);

        /////////GET MODEL STATE ///////////
        if (simulated_env_)
        {
            gazebo_client_ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo_launch/get_model_state");
            model_state_.request.model_name = model_name;

            while(!gazebo_client_.call(model_state_));

            ///////SET MODEL STATE//////////
            geometry_msgs::Pose start_pose;
            start_pose.position.x = 0.0;
            start_pose.position.y = 0.0;

            tf::Quaternion q(0, 0, 0, 0);
            q.setRPY(0, 0, -1.5707);

            start_pose.orientation.x = (double) q.x();
            start_pose.orientation.y = (double) q.y();
            start_pose.orientation.z = (double) q.z();
            start_pose.orientation.w = (double) q.w();

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = model_name;
            modelstate.pose = start_pose;

            ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo_launch/set_model_state");
            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;
            while(!client.call(setmodelstate));

            if (!gazebo_client_.call(model_state_))
            {
                ROS_ERROR("fuck you gazebo");
            }

            //rosservice call 0, 0 pi/2
            geometry_msgs::Pose pose;
            pose = model_state_.response.pose;
            pose_.x = pose.position.x;
            pose_.y = pose.position.y;

            pose_.theta = map_utils::convertQuatToAngle(pose.orientation);
        
        }
        
        else {
            IMU_client_ = nh.serviceClient<finderbot::getImuPose>("finderbot_IMU_pose"); 
            if (IMU_client_.call(imu_pose_))
            {
                pose_ = imu_pose_.response.pose;
            }
        }

        ROS_INFO("SLAM_NODE: initial pose = (%f, %f, %f)",pose_.x, 
                                                         pose_.y,
                                                         pose_.theta);
        pose_publisher_.publish(pose_);
    }

    void generateParticles()
    {
        std::default_random_engine generator;
        std::normal_distribution<double> x_distribution(pose_.x, std_dev_);
        std::normal_distribution<double> y_distribution(pose_.y, std_dev_);
        std::normal_distribution<double> theta_distribution(pose_.theta, std_dev_);

        particles_.clear();
        for (int i = 0; i < num_particles_; i++)
        {
            finderbot::Pose particle;

            particle.x = (float) x_distribution(generator);
            particle.y = (float) y_distribution(generator);
            particle.theta = (float) theta_distribution(generator);

            particles_.push_back(particle);
        }
    }

    finderbot::Pose getMLPose()
    {
        finderbot::Pose max_pose;
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
        return max_pose;
    }

    void generateSLAMPose()
    {
        if (!new_pf_data) return;

        if(simulated_env_)
        {
            geometry_msgs::Pose pose;
            if (gazebo_client_.call(model_state_))
            {
                 pose = model_state_.response.pose;
                 pose_.x = pose.position.x;
                 pose_.y = pose.position.y;
                 tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                 pose_.theta = map_utils::convertQuatToAngle(q);
            }
            else ROS_ERROR("boooo gazebo");
        }
        else
        {
            if(IMU_client_.call(imu_pose_))
            {
                pose_ = imu_pose_.response.pose;
            }
        }
        
        //fill particles vector by sampling num_particles times from N(pose, std_dev)
        generateParticles();
        
        //loop through particles
        //select particle with highest pose, update pose
        finderbot::Pose max_pose = getMLPose();
        
        pose_ = max_pose;
        
        //publish data to global map builder
        //and to path execution node
        pose_publisher_.publish(pose_);
    }

    //update local pf data
    void handlePFData(const finderbot::PF_Input pfData)
    {
        pfInput.map_width = pfData.map_width;
        pfInput.map_height = pfData.map_height;
        pfInput.map_resolution = pfData.map_resolution;
        pfInput.log_odds.assign(pfData.log_odds.begin(), pfData.log_odds.end());
        pfInput.scan_ranges.assign(pfData.scan_ranges.begin(), pfData.scan_ranges.end());
        pfInput.scan_angles.assign(pfData.scan_angles.begin(), pfData.scan_angles.end());
        new_pf_data = true;
    }

    //spi node will publish pose data calculated from IMU,
    //handleIMU Pose will provide pose to sample from 
    // void handleIMUPose(const finderbot::Pose)
    // {

    // }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "SLAM");
    ros::NodeHandle nh;

    std::string world_frame_id;
    std::string laser_frame_id;
    std::string model_name;
    int num_particles;
    double std_dev;
    bool simulated_env;
    nh.param<std::string>("world_frame_id", world_frame_id, "world");
    nh.param<std::string>("laser_frame_id", laser_frame_id, "laser_frame");
    nh.param<std::string>("model_name", model_name, "Finderbot_Lidar");
    nh.param<int>("pf_num_particles", num_particles, 10000);
    nh.param<double>("pf_std_dev", std_dev, 0.1);
    nh.param<bool>("simulated_env", simulated_env, false);
    
    if (simulated_env) ROS_INFO("USING SIMULATED DATA");

    SLAM slamma_jamma(simulated_env, num_particles, std_dev, model_name);

    ros::Subscriber pf_handler = nh.subscribe<finderbot::PF_Input>("SLAM_pf", 1, &SLAM::handlePFData, &slamma_jamma);
    
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        slamma_jamma.generateSLAMPose();

        ros::spinOnce();

        loop_rate.sleep();
    }

}