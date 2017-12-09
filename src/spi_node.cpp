#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

#include <finderbot/spi.h>
#include <finderbot/Pose.h>
#include <finderbot/getImuPose.h>
#include <inttypes.h>

//class to synchronize communication with PCB over SPI
//spi functions in spi.h
class SpiSync{

    ros::Time prev_command_time_;
    ros::Time prev_IMU_time_;

    finderbot::Pose pose_;
    uint64_t spi_delay_;

    uint64_t IMU_delay_;

    double v_x_;
    double v_y_;

    void spiDelay()
    {
        uint64_t time_since_prev = ros::Time::now().toNSec() - prev_command_time_.toNSec();
        
        if(time_since_prev > spi_delay_)
        {
            ros::Duration(0, spi_delay_ - time_since_prev).sleep();
        }
    }

public:

    SpiSync(uint64_t spi_delay, uint64_t IMU_delay)
    {
        pose_.x = 0.0;
        pose_.y = 0.0;
        pose_.theta = 0.0;

        spi_delay_ = spi_delay;
        IMU_delay_ = IMU_delay;
        prev_command_time_ = ros::Time::now();
        prev_IMU_time_ = prev_command_time_;
    }

    //updates pose data 
    void getImuData()
    {
        spiDelay();
        float accel = readAccel('X');
        spiDelay();
        //IMU returns absolute orientation
        //TODO initial offset
        pose_.theta = readAccel('T');

        double a_x = accel * std::cos(pose_.theta);
        double a_y = accel * std::sin(pose_.theta);
        double dt = ros::Time::now().toSec() - prev_IMU_time_.toSec();

        prev_IMU_time_ = ros::Time::now();
        prev_command_time_ = prev_IMU_time_;

        v_x_ = v_x_ + a_x*dt;
        v_y_ = v_y_ + a_y*dt;

        pose_.x = pose_.x + v_x_*dt;
        pose_.y = pose_.y + v_y_*dt;

    }

    //TODO dont do all this in the handler?
    void handleCommandVel(const geometry_msgs::Twist new_cmd)
    {
        spiDelay();
        if (new_cmd.linear.x != 0)
        {
            readWriteHeading(pose_.theta);
            prev_command_time_ = ros::Time::now();

            spiDelay();
            readWriteMotor('f');
        }

        else if (new_cmd.angular.z != 0)
        {
            if(new_cmd.angular.z > 0) readWriteMotor('L');
            else readWriteMotor('R');
        }

        else readWriteMotor('h');

        prev_command_time_ = ros::Time::now();

        return;
    }

    //subscribe to pose data corrected by SLAM
    void handleSLAMPose(const finderbot::Pose slam_pose)
    {
        pose_ = slam_pose;
    }

    //service called by SLAM node
    //TODO make this a srv and add to SLAM
    bool sendIMUPose(finderbot::getImuPose::Request& req, 
                     finderbot::getImuPose::Response& res)
    {
        //return whatever is in Pose
        res.pose = pose_;
        return true;
    }

    //this needs to be public since IMU reads need to be done regularly 
    //and aren't controlled by a callback
    bool IMU_delay_elapsed()
    {
	ROS_INFO("delay ns = %llu, diff_t = %llu",IMU_delay_,  ros::Time::now().toNSec() - prev_IMU_time_.toNSec());

        return (ros::Time::now().toNSec() - prev_IMU_time_.toNSec()) > IMU_delay_;
    }
};


//receive command velocities
//take in slam poses
//read from IMU
//send pose to SLAM

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "SPI_Transmitter");
    ros::NodeHandle nh;

    int channel = 0;
    wiringPiSPISetup(channel, 4000000);

    bool simulated;
    int spi_delay_ms;
    int IMU_delay_ms;

    nh.param<bool>("simulated_env", simulated, true);
    nh.param<int>("spi_delay_ms", spi_delay_ms, 5);
    nh.param<int>("IMU_delay_ms", IMU_delay_ms, 100);

    uint64_t spi_delay_ns = spi_delay_ms*1e6;
    uint64_t IMU_delay_ns = IMU_delay_ms*1e6;

    SpiSync spi_sync(spi_delay_ms, IMU_delay_ms);
    //TODO service to get Pose from SLAM 
    //TODO subscribe to SLAM pose
    ros::Subscriber cmd_handler = nh.subscribe<geometry_msgs::Twist>("finderbot_cmd_vel", 1, &SpiSync::handleCommandVel, &spi_sync);
    ros::Subscriber slam_handler = nh.subscribe<finderbot::Pose>("finderbot_pose", 1, &SpiSync::handleSLAMPose, &spi_sync);

    ros::ServiceServer service = nh.advertiseService("finderbot_IMU_pose", &SpiSync::sendIMUPose, &spi_sync);
    while (ros::ok())
    {
        //only read from IMU if enough time elapsed
        //also if its not simulated, cus then it doesnt exist
        if (spi_sync.IMU_delay_elapsed() && !simulated)
        {
	    ROS_INFO("Get IMU data");
            spi_sync.getImuData();
        }

        ros::spinOnce();
    }
    

    return 0;
}
