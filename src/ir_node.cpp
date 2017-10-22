#include "ros/ros.h"
#include "finderbot/Ir.h"
#include "finderbot/ir_sensor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ir_node");

    ros::NodeHandle handler;

    ros::Publisher ir_pub = handler.advertise<finderbot::Ir>("ir_data",1000);

    ros::Rate loop_rate(10);

    int count = 0;
    InfraredSensor ir_1;
    ir_1.set_data(10);

    while(ros::ok()){
       // define msg here 
        finderbot::Ir msg;
        msg.id = 0;
        msg.value = ir_1.get_data();
        ir_pub.publish(msg);
        
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
