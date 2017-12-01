#include "ModuleConnector.hpp"
#include "X4M300.hpp"
#include "xtid.h"
#include <iostream>
#include <unistd.h>

/** \example presence_single.cpp
 * this is a small ModuleConnector usage example
 */

using namespace XeThru;


int handle_error(std::string message)
{
    std::cerr << "ERROR: " << message << std::endl;
    return 1;
}

int presence_single(const std::string & device_name)
{
	int n;
	float prev_distance;
	int confidence;
    const unsigned int log_level = 0;
    bool state;
    ModuleConnector mc(device_name, log_level);
    X4M300 & x4m300 = mc.get_x4m300();
    sleep(3);
    XeThru::PresenceSingleData presence_single;
	x4m300.set_baudrate(XTID_BAUDRATE_115200);
	//sleep(3);
    std::cout << "Stop the module" << std::endl;
    x4m300.set_sensor_mode(XTID_SM_STOP, 0);

	//set baud rate
	
	
	//ping the module
	unsigned int pong = 0;
    int status = x4m300.ping(&pong);
    if(status != 0) {
        std::cout << "Something went wrong - error code: " << status << std::endl;
        return status;
    }
    std::cout << "pong: " << std::hex << pong << std::endl;
   
	std::cout << "Flushing any old data." << std::endl;	//flushing old data
    while (x4m300.peek_message_presence_single())
        x4m300.read_message_presence_single(&presence_single);

    std::cout << "Load presence profile" << std::endl;
    const unsigned int ProfileID = 0x014d4ab8;
    if (x4m300.load_profile(ProfileID)) {
        return handle_error("load_profile failed");
    }
	
    std::cout << "Turn on presence single packages" << std::endl;
    if (x4m300.set_output_control(XTS_ID_PRESENCE_SINGLE, XTID_OUTPUT_CONTROL_ENABLE)) {
        return handle_error("set output controll failed");
    }
	if (x4m300.set_output_control(XTS_ID_PRESENCE_MOVINGLIST, 0)) {
        return handle_error("set output controll failed");
    }
    
    std::cout << "Setting detection zone 0.5 - 9 " << std::endl;
	x4m300.set_detection_zone(0.5, 9);
	std::cout << "Setting sensitivity 5" << std::endl;
    x4m300.set_sensitivity(5);
    
    // start the module and profile
    std::cout << "Set the module in RUN state" << std::endl;
    if (x4m300.set_sensor_mode(XTID_SM_RUN, 0)) {
        return handle_error("Set sensor mode to running failed");
    }

    std::cout << "Block until first package of presence arrives" << std::endl;
    std::cout << "This may take some time..." << std::endl;
	state = 0;
	confidence = 0;
    n=0;
    while (n < 150) {
		sleep(0.1);
		while (x4m300.peek_message_presence_single()){
			if (x4m300.read_message_presence_single(&presence_single)) {
				return handle_error("set output controll failed");
			}
			if (n==0){
				std::cout << "Get the data,but need tracking...Please wait..." << std::endl;
			}
			if(presence_single.presence_state != 2){ 
			std::cout << "frame_counter:  " << presence_single.frame_counter << std::endl;
			std::cout << "presence_state: " << presence_single.presence_state << std::endl;
			std::cout << "distance:       " << presence_single.distance << std::endl;
			std::cout << "direction:      " << static_cast<unsigned int>(presence_single.direction) << std::endl;
			std::cout << "signal_quality: " << presence_single.signal_quality << std::endl;
			n+=1;
			if (state == 0 && (presence_single.distance - prev_distance <= 1) && (presence_single.signal_quality > 1) && presence_single.presence_state == 1)
				confidence += 1;
			else if (state == 0)
				confidence = 0;
			if (state == 1 && confidence == 2 && (presence_single.distance - prev_distance <= 1) && (presence_single.signal_quality > 2) && presence_single.presence_state == 1)
				confidence += 1;
			else if (state == 1 && confidence == 2)
				confidence = 0;
			else if (state == 1 && confidence == 3)
				confidence -= 1;
			if (confidence == 2)
				state = 1;
			if (confidence == 0)
				state = 0;
	 
			if (state == 1)
				std::cout << "Vital signal found! Distance: " << presence_single.distance << std::endl;
			else
				std::cout << "No vital signal found... " << std:: endl;
			prev_distance = presence_single.distance;
			}
		}	
    }
   // std::cout << "Set the module in IDLE state" << std::endl;
   // if (x4m300.set_sensor_mode(XTID_SM_IDLE, 0)) {
   //     return handle_error("set output controll failed");
   // }
   // sleep(3);
   // std::cout << "Set the module in RUN state" << std::endl;
   // if (x4m300.set_sensor_mode(XTID_SM_RUN, 0)) {
   //    return handle_error("set output controll failed");
   // }
    /*
    n=0;
    while (n < 50) {
		sleep(0.1);
		while (x4m300.peek_message_presence_single()){
			if (x4m300.read_message_presence_single(&presence_single)) {
				return handle_error("set output controll failed");
			}
			if (n==0)
				std::cout << "Get the first data,need tracking......" << std::endl;
			if(presence_single.presence_state != 2){ 
			std::cout << "frame_counter:  " << presence_single.frame_counter << std::endl;
			std::cout << "presence_state: " << presence_single.presence_state << std::endl;
			std::cout << "distance:       " << presence_single.distance << std::endl;
			std::cout << "direction:      " << static_cast<unsigned int>(presence_single.direction) << std::endl;
			std::cout << "signal_quality: " << presence_single.signal_quality << std::endl;
			
			n+=1;
			}
		}	
    }
    */
    std::cout << "Set the module in STOP state" << std::endl;
    if (x4m300.set_sensor_mode(XTID_SM_STOP, 0)) {
        return handle_error("set output controll failed");
    }

    return 0;
}


int main()
{

    const std::string device_name = "/dev/ttySAC0";

    return presence_single(device_name);
}
