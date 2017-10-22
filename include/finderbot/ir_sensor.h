/* Part Datasheet https://www.sparkfun.com/datasheets/Components/GP2Y0A21YK.pdf
 * 
 * From 10 Data points on 2 sensors, our regression results were as follows:
 * 	Sensor 5Z:	-9.47x + 717
 * 	Sensor IZ:	-7.93x + 659
 *
 * From all 20 Data points from both sensors, our total regression was:
 * 	Total:		-8.7x+688
 *
 */
class InfraredSensor
{
	int data;

public:

	InfraredSensor() {}
	
	int get_data(){
		return data;
	}

	void set_data(int data_in){
		data = data_in;
	}
 //  public:
 //  	InfraredSensor(const int _DATA) : DATA(_DATA) {}
	// // enables the infrared sensor and starts measuring distance.
	// void init() {
	// 	pinMode(DATA, INPUT);
	// }
	// // gets the measured distance of the the obstacle using regression.
	// int get_distance_in_mm() {
	// 	// voltage reading from IR sensor
	// 	int voltage = analogRead(DATA);
	// 	// distance in mm using our regression
	// 	return (int)(10 * (voltage - 688) / (-87));
	// }
	// // returns 1 if out of the threshold, else returns 0
	// int out_of_threshold() {
	// 	if (get_distance() > 15 && get_distance() < 25) {
	// 		return 1
	// 	}
	// 	return 0;
	// }
};