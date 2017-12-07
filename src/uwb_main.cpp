#include <finderbot/uwb.h>
#include <finderbot/UWB/ModuleConnector.hpp>
#include <ctime>
#include <iostream>
#include <unistd.h>

int main(){

	const std::string device_name = "/dev/ttySAC0";
    const unsigned int log_level = 0;

	XeThru::ModuleConnector mc(device_name, log_level);
    //X4M300 &x4m300 = mc.get_x4m300();
	XeThru::Uwb uwb_(mc);
	uwb_.uwb_setup();
	uwb_.uwb_get_data();
	sleep(5);
	uwb_.uwb_get_data();
	sleep(5);
	uwb_.uwb_get_data();
	sleep(5);
	uwb_.uwb_stop();

	return 0;
}
