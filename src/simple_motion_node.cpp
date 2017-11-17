#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//Literally just tell gazebo finderbot to move forward

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_motion");
    ros::NodeHandle nh;

    bool sim;
    nh.param<bool>("sim", sim, false);
    
    geometry_msgs::Vector3 vel;
    vel.x = -0.03;
    vel.y = 0;
    vel.z = 0;

    geometry_msgs::Twist cmd;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("finderbot_cmd_vel",1,true);

    while(ros::ok())
    {
        cmd.linear = vel;
        pub.publish(cmd);
    }
}