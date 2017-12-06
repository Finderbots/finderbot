#include <ros/ros.h>
#include <ros/console.h>

#include <finderbot/wiringPi.h>
#include <finderbot/wiringPiSPI.h>

typedef struct {
    uint8_t zero : 8;
    uint8_t one : 8;
    uint8_t two : 8;
    uint8_t three : 8; 
}Bytes;

typedef union {
    float data;
    Bytes bytes;
}float_bytes; 


float readWriteAccelPCB(unsigned char* buf)
{
    int channel = 0;
    
    float_bytes bytes_to_float;
    //send start bit
    int ret = wiringPiSPIDataRW (channel, buf, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }

    //send desired axis
    ret = wiringPiSPIDataRW(channel, buf+1, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    //check that start bit returned ack
    if(buf[1] != '!') ROS_ERROR("No Ack from PCB");

    //request first data byte, read calibration byte
    ret = wiringPiSPIDataRW(channel, buf+2, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    ROS_INFO("calib = %i", buf[2]);

    //request second data byte read first data byte
    ret = wiringPiSPIDataRW(channel, buf+3, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    bytes_to_float.bytes.zero = buf[3];

    ret = wiringPiSPIDataRW(channel, buf+4, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    bytes_to_float.bytes.one = buf[4];

    ret = wiringPiSPIDataRW(channel, buf+5, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    bytes_to_float.bytes.two = buf[5];

    ret = wiringPiSPIDataRW(channel, buf+6, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    bytes_to_float.bytes.three = buf[6];
    
    ret = wiringPiSPIDataRW(channel, buf+7, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[7] != 'd') ROS_ERROR("No Ack from PCB");

    return bytes_to_float.data;
}

void printBuf(unsigned char* buf)
{
    for (int i = 0; i < 7; i++)
    {
        ROS_INFO("SPI: %c  %i", buf[i], (uint8_t)buf[i]);
    }
}



int main(int argc, char** argv)
{   
    ros::init(argc, argv, "SPI_Transmitter");
    ros::NodeHandle nh;

    int channel = 0;
    wiringPiSPISetup(channel, 4000000);

    //run at 100Hz
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        unsigned char buf[9] = "sX1234e_";

        ROS_INFO("lin accel(x) = %f", readWriteAccelPCB(buf));

       // memcpy(buf,"sy1234e_", 9);

        //ROS_INFO("lin accel(x) = %f", readWriteAccelPCB(buf));

        //memcpy(buf,"sT1234e_", 9);

        //ROS_INFO("lin accel(x) = %f", readWriteAccelPCB(buf));        

        loop_rate.sleep();   
    }

    return 0;
}
