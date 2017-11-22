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
	int pin;
  public:
  	InfraredSensor() {}
	
	// enables the infrared sensor and starts measuring distance.
	void init();
	
	// gets the measured distance of the the obstacle using regression.
	int get_distance_mm();
	
	// returns 1 if out of the threshold, else returns 0
	int out_of_threshold(int dist);
};
