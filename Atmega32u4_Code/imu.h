#ifndef IMU_h
#define IMU_h

// #include "FIMU_ADXL345.h"
// #include "FIMU_ITG3200.h"
// #include "FreeSixIMU.h"
#include "Adafruit_BNO055.h"
//#include "i2c_master.h"

//extern FreeSixIMU sixDOF;
extern Adafruit_BNO055 bno;

/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
extern uint8_t roll;
extern uint8_t pitch;
extern uint8_t yaw;


extern float angles[5];


//Location PID CONTROL 
extern float Kp;
extern float Ki;
extern float Kd;

void updateAngle(void);
void init_imu();

#endif