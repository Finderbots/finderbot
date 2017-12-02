#include "imu.h"

FreeSixIMU sixDOF = FreeSixIMU();

float angles[5];

/* odometry data read from IMU. Updated by IMURead. Sent via SPI to XU4 */
uint8_t roll = 0;
uint8_t pitch = 0;
uint8_t yaw = 0;

//Location PID CONTROL 
 float Kp = 0.5;
 float Ki = 0.05;
 float Kd = 0.4;

void updateAngle(void) {
    //PORTD |= _BV(PD2); //debugging
    sixDOF.getYawPitchRoll(angles);
    // prevAngles[prevAngleI] = angles[1];
    // prevAngleI = (prevAngleI + 1) % AvgAngles;
    // float sum = 0;
    // for (int i = 0; i < AvgAngles; i++)
    //     sum += prevAngles[i];
    // currAngle = sum / AvgAngles;
    // prevAngle = currAngle;
    //PORTD &= ~(_BV(PD2)); //debugging
}
