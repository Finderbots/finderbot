#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <string>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_frame_transformer");
    ros::NodeHandle nh;

    std::string world_frame_id;
    std::string laser_frame_id;
    std::string model_name;
    nh.param<std::string>("world_frame_id", world_frame_id, "world");
    nh.param<std::string>("laser_frame_id", laser_frame_id, "laser_frame");
    nh.param<std::string>("model_name", model_name, "Finderbot_Lidar");

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo_launch/get_model_state");
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = model_name;
    
    tf::Transform transform;
    static tf::TransformBroadcaster br;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (client.call(model_state))
        {
            geometry_msgs::Pose pose = model_state.response.pose;
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            transform.setRotation(q);

            tf::Vector3 position(pose.position.x, pose.position.y, 0);
            transform.setOrigin(position);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                                    world_frame_id, laser_frame_id));
        }

        loop_rate.sleep();

    }
    

}