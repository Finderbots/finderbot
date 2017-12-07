#include <finderbot/uwb.h>
#include <iostream>
#include <unistd.h>

/** \example presence_single.cpp
 * this is a small ModuleConnector usage example
 */
namespace XeThru{

int Uwb::handle_error(std::string message)
{
    std::cerr << "ERROR: " << message << std::endl;
    return 1;
}


int Uwb::uwb_setup()
{
	
	bool init_done = 0;
    
    sleep(3);
	//set baud rate
    XeThru::PresenceSingleData presence_single;
	x4m300.set_baudrate(XTID_BAUDRATE_115200);

    std::cout << "Stop the module" << std::endl;
    x4m300.set_sensor_mode(XTID_SM_STOP, 0);	
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
	std::cout << "Initializing...Will take about 2 minutes..." << std::endl;
	while (init_done != 1){
		while (x4m300.peek_message_presence_single()){
			x4m300.read_message_presence_single(&presence_single);
			if (presence_single.presence_state != 2){
				init_done = 1;
				break;
			}
		}
	}
	std::cout << "Initializing Done!" << std::endl;
	return 0;
}

int Uwb::uwb_get_data()
{	
	confidence = 0;
	state = 0;
	XeThru::PresenceSingleData presence_single;
    int n=0;
	while (x4m300.peek_message_presence_single())
        x4m300.read_message_presence_single(&presence_single);
	
    while (n < 5) {	
		//sleep(0.1);
		while (x4m300.peek_message_presence_single() && n < 5){
			if (x4m300.read_message_presence_single(&presence_single)) {
				return handle_error("set output controll failed");
			}
			std::cout << "frame_counter:  " << presence_single.frame_counter << std::endl;
			std::cout << "presence_state: " << presence_single.presence_state << std::endl;
			std::cout << "distance:       " << presence_single.distance << std::endl;
			std::cout << "direction:      " << static_cast<unsigned int>(presence_single.direction) << std::endl;
			std::cout << "signal_quality: " << presence_single.signal_quality << std::endl;
			n+=1;
			if (state == 0 && (presence_single.distance - prev_distance <= 0.5) && (presence_single.signal_quality > 1) && presence_single.presence_state == 1)
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
			prev_distance = presence_single.distance;
		}
	}
	if (state == 1)
		std::cout << "Vital signal found! Distance: " << presence_single.distance << std::endl;
	else
		std::cout << "No vital signal found... " << std:: endl;
						  
    return 0;
}

int Uwb::uwb_stop()
{
	std::cout << "Set the module in STOP state" << std::endl;
    if (x4m300.set_sensor_mode(XTID_SM_STOP, 0)) {
        return handle_error("set output controll failed");
    }
	return 0;
}
}

