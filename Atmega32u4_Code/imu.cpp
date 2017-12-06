#include "imu.h"

// FreeSixIMU sixDOF = FreeSixIMU();
Adafruit_BNO055 bno = Adafruit_BNO055(55);

float angles[5];

/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
float_bytes lin_accel;
float_bytes y_accel;
float_bytes heading;
uint8_t sys_calib;



void init_imu() {
    //cli();
    bno.begin();
    
     delay(1000);

    /* Use external crystal for better accuracy */
    bno.setExtCrystalUse(true);
    //sei();
}

void update_vals(void) {

    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    sys_calib = sys;

    if(gyro >= 2) { //check for valid data
        imu::Vector<3> accelerations = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        lin_accel.num_float = accelerations.x();
        y_accel.num_float = accelerations.y();
    }
    if(mag >= 2) { //check for valid data
        sensors_event_t event;
        bno.getEvent(&event);
        heading.num_float = event.orientation.x;
    }

}
