#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>

int getch()
{
      static struct termios oldt, newt;
      tcgetattr( STDIN_FILENO, &oldt);           // save old settings
      newt = oldt;
      newt.c_lflag &= ~(ICANON);                 // disable buffering      
      tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

      int c = getchar();  // read character (non-blocking)

      tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
      return c;
}

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

    geometry_msgs::Vector3 turn;
    turn.x  = 0;
    turn.y = 0;
    turn.z = 0.03;

    geometry_msgs::Twist cmd;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("finderbot_cmd_vel",1,true);

    while(ros::ok())
    {
        int c = getch();

        cmd.linear.x = 0;
        cmd.angular.z = 0;

        if (c == 'w')
        {
            // ROS_INFO("forward");
            cmd.linear.x = -0.03; //forward is negative, deal with it.
        }

        else if (c == 'a')
        {
            // ROS_INFO("left");
            cmd.angular.z = 0.05;
        }

        else if (c == 'd')
        {
            // ROS_INFO("back");
            cmd.angular.z = -0.05;
        }

        pub.publish(cmd);
    }
}