#include <finderbot/IR_Sensor.h>

//TODO GPIO for XU4
void InfraredSensor::init() {
	// pinMode(DATA, INPUT);
}

int InfraredSensor::get_distance_mm() {
	// voltage reading from IR sensor
	// int voltage = analogRead(DATA);
    int voltage = 0;
	// distance in mm using our regression
	return (int)(10 * (voltage - 688) / (-87));
}

int InfraredSensor::out_of_threshold(int dist) {
	if (dist > 15 && dist < 25) {
		return 1;
	}
	return 0;
}
