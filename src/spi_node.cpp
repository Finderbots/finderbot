#include <ros/ros.h>
#include <ros/console.h>

#include <finderbot/wiringPi.h>
#include <finderbot/wiringPiSPI.h>


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "SPI Transmitter");
    ros::NodeHandle nh;

    int channel = 0;
    wiringPiSPISetup(channel, 4000000);

    while (ros::ok())
    {

    }

    return 0;
}