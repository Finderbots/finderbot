#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

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
    if(buf[7] != '!') ROS_ERROR("No Ack from PCB");

    return bytes_to_float.data;
}

void printBuf(unsigned char* buf, int size)
{
    for (int i = 0; i < size; i++)
    {
        ROS_INFO("SPI: %c  %i", buf[i], (uint8_t)buf[i]);
    }
}


void readWriteHeading(float heading)
{
    int channel = 0;

    float_bytes bytes_to_float;
    bytes_to_float.data = heading;

    unsigned char buf[9] = "sH0000e_";

    buf[2] = bytes_to_float.bytes.zero;
    buf[3] = bytes_to_float.bytes.one;
    buf[4] = bytes_to_float.bytes.two;
    buf[5] = bytes_to_float.bytes.three;
    
    //send s
    int ret = wiringPiSPIDataRW (channel, buf, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }

    //send C
    ret = wiringPiSPIDataRW(channel, buf+1, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    //check that start bit returned ack
    if(buf[1] != '!') ROS_ERROR("No Ack to start");

    //send data1
    ret = wiringPiSPIDataRW(channel, buf+2, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[2] != '!')
    {
        ROS_ERROR("No Ack to H");
    }

    //send data 2
    ret = wiringPiSPIDataRW(channel, buf+3, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if((uint8_t)buf[3] != bytes_to_float.bytes.zero)
    {
        ROS_ERROR("Incorrect byte 1");
    }

    //send data 3
    ret = wiringPiSPIDataRW(channel, buf+4, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[4] != bytes_to_float.bytes.one)
    {
        ROS_ERROR("Incorrect byte 2");
    }

    //send data 4
    ret = wiringPiSPIDataRW(channel, buf+5, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[5] != bytes_to_float.bytes.two)
    {
        ROS_ERROR("Incorrect byte 3");
    }

    //send end
    ret = wiringPiSPIDataRW(channel, buf+6, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[6] != bytes_to_float.bytes.three)
    {
        ROS_ERROR("Incorrect byte 4");
    }

    //send _
    ret = wiringPiSPIDataRW(channel, buf+7, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[7] != 'd')
    {
        ROS_ERROR("Incorrect byte 4");
    }

}



void readWriteMotor(char cmd)
{
    int channel = 0;
    
    unsigned char buf[6] = "sC0e_";
    //send start bit
    buf[2] = cmd;

    int ret = wiringPiSPIDataRW (channel, buf, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }

    //send C
    ret = wiringPiSPIDataRW(channel, buf+1, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    //check that start bit returned ack
    if(buf[1] != '!') ROS_ERROR("No Ack to start");

    //send fBhLR
    unsigned char cmd = buf[2];
    ret = wiringPiSPIDataRW(channel, buf+2, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[2] != '!')
    {
        ROS_ERROR("No Ack to C");
    }

    //send end byte
    ret = wiringPiSPIDataRW(channel, buf+3, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[3] != cmd) ROS_ERROR("SPI: sent: %c, got %c back", cmd, buf[3]); 

    //send empty
    ret = wiringPiSPIDataRW(channel, buf+4, 1);
    if (ret < 0)
    {
        ROS_ERROR("SPI: ret = %i\n", ret);
        exit(1);
    }
    if(buf[4] != 'd') ROS_ERROR("No end ACK");

}

void handleCommandVel(const geometry_msgs::Twist new_cmd)
{
    if (new_cmd.linear.x != 0)
    {
        readWriteMotor('f');
    }
    else if (new_cmd.angular.z != 0)
    {
        if(new_cmd.angular.z > 0) readWriteMotor('L');
        else readWriteMotor('R');
    }

    else readWriteMotor('h');

    return;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "SPI_Transmitter");
    ros::NodeHandle nh;

    int channel = 0;
    wiringPiSPISetup(channel, 4000000);

    //run at 100Hz
    ros::Rate loop_rate(100);

    //TODO service to get Pose from SLAM 
    //TODO subscribe to SLAM pose

    ros::Subscriber cmd_handler = nh.subscribe<geometry_msgs::Twist>("finderbot_cmd_vel", 1, handleCommandVel);

    ros::spin();

    return 0;
}