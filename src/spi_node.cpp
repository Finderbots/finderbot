#include <ros/ros.h>
#include <ros/console.h>

#include <finderbot/wiringPi.h>
#include <finderbot/wiringPiSPI.h>

void readWritePCB(unsigned char* buf)
{
    int channel = 0;
    for (int i = 0; i < 8; i++)
    {
        int ret = wiringPiSPIDataRW (channel, buf+i, 1);
        if (ret < 0)
        {
            ROS_ERROR("SPI: ret = %i\n", ret);
            exit(1);
        }

    }
}

void printBuf(unsigned char* buf)
{
    for (int i = 0; i < 8; i++)
    {
        ROS_INFO("SPI: %c  %i", buf[i], (uint8_t)buf[i]);
    }
}



int main(int argc, char** argv)
{   
    ros::init(argc, argv, "SPI Transmitter");
    ros::NodeHandle nh;

    int channel = 0;
    wiringPiSPISetup(channel, 4000000);

    //run at 100Hz
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        unsigned char buf[9] = "sirpy<>e";

        readWritePCB(buf);

        loop_rate.sleep();   
    }

    return 0;
}