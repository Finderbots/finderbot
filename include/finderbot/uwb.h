#ifndef UWB_H_
#define UWB_H_

#include <finderbot/UWB/ModuleConnector.hpp>
#include <finderbot/UWB/X4M300.hpp>
#include <finderbot/UWB/xtid.h>
#include <iostream>
#include <unistd.h>

/** \example presence_single.cpp
 * this is a small ModuleConnector usage example
 */

namespace XeThru{

class Uwb{
	public:
		Uwb(ModuleConnector& mc_) : x4m300(mc_.get_x4m300()){};
		float prev_distance;
		int confidence;
		bool state;
		X4M300& x4m300;
		int uwb_setup();
		int uwb_get_data();
		int uwb_stop();
	private:
		int handle_error(std::string message);
};

}
#endif
