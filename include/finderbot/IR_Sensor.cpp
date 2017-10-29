#include "IR_Sensor.h"

void InfraredSensor::init() {
	pinMode(DATA, INPUT);
}

int InfraredSensor::get_distance_mm() {
	// voltage reading from IR sensor
	int voltage = analogRead(DATA);
	// distance in mm using our regression
	return (int)(10 * (voltage - 688) / (-87));
}

int InfraredSensor::out_of_threshold() {
	if (get_distance_mm() > 15 && get_distance_mm() < 25) {
		return 1;
	}
	return 0;
}
