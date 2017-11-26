#include <CommunicationUtils.h>

#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>

#define DEBUG
#ifdef DEBUG
#include <DebugUtils.h>
#endif


FreeSixIMU sixDOF = FreeSixIMU();


const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

// time vars
int currTime = 0; 
int prevTime = 0; 



float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
  float Lp = 0.5;
  float Li = 0.05;
  float Ld = 0.4;
  float offsetLoc = 0;
  float pT,iT,dT = 0;
  float errorS = 0;
  float prevE = 0;

void setup() {

  
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

  delay(5);
  sixDOF.init(); //Begin the IMU
  delay(5);

}


void loop() {

  updateAngle();
  Serial.println(currAngle);
  delay(500);
  
}

  
void updateAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}

