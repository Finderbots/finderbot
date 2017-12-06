#include "imu.h"

// FreeSixIMU sixDOF = FreeSixIMU();
Adafruit_BNO055 bno = Adafruit_BNO055(55);

float angles[5];

/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
float_bytes lin_accel;
float_bytes y_accel;
float_bytes heading;
uint8_t sys_calib;


//Location PID CONTROL 
float_bytes Kp;
float_bytes Ki;
float_bytes Kd;


void init_imu() {
    //cli();
    bno.begin();
    
     delay(1000);

    /* Use external crystal for better accuracy */
    bno.setExtCrystalUse(true);
    //sei();
}

void update_vals(void) {

    sensors_event_t event;
    // cli();
    bno.getEvent(&event);
    heading.num_float = event.orientation.x;
    // sei();


    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    // cli();
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    sys_calib = sys;
    // sei();
}
