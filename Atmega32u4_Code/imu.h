#ifndef IMU_h
#define IMU_h

// #include "FIMU_ADXL345.h"
// #include "FIMU_ITG3200.h"
// #include "FreeSixIMU.h"
#include "Adafruit_BNO055.h"
//#include "i2c_master.h"

//extern FreeSixIMU sixDOF;
extern Adafruit_BNO055 bno;

extern float angles[5];

typedef union {
	float num_float;
	struct {
		uint8_t first : 8;
		uint8_t second : 8;
		uint8_t third : 8;
		uint8_t fourth : 8;
	} bytes;

} float_bytes;


/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
extern float_bytes lin_accel;
extern float_bytes y_accel;
extern float_bytes heading;
extern uint8_t sys_calib;
extern uint8_t accel_calib;
extern uint8_t mag_calib;

void update_vals(void);
void init_imu();

#endif