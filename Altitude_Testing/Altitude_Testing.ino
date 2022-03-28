#define LOCALPRESSURE 1016.8  // used to calculate altitude

#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>

LSM6 imu;  // sensor stuff
LPS ps;
float initAlt = 0;
float currentAlt = 10000;
float checkAlt1 = 10000;
float checkAlt2 = 10000;
int landed = 0;
int altRange = 1; //Amount of meters the rocket can be above the starting value. 

void setup() {
  // put your setup code here, to run once:
  // ----- LSM6DS33 IMU SETUP -----
  if (!imu.init())
    Serial.println("LSM6DS33 initialization failed");
  else
    Serial.println("LSM6DS33 initialized");

  imu.enableDefault();
  // ODR_XL = 1000 (1.66 kHz (high performance))
  // FS_XL = 01 (+/-16 g full scale)
  // BW_XL = 00 (400 Hz) (Overridden when XL_BW_SCAL_ODR=0)
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  // ODR = 1000 (1.66 kHz (high performance))
  // FS_G = 11 (2000 dps)
  imu.writeReg(LSM6::CTRL2_G, 0b10001100);

  delay(500);
  initAlt = ps.readPressureMillibars();
  initAlt = ps.pressureToAltitudeMeters(initAlt);

  Serial.println("Startup complete");
}

void loop() {
  // Check current altitude. If current altitude is close enough to intial value, then enter landing detection.
  // Get altitude, wait 2 seconds, get altitude again, and compare. If the 2 values are close, we have probably landed. 
  while(landed == 0){
    currentAlt = ps.readPressureMillibars();
    currentAlt = ps.pressureToAltitudeMeters(currentAlt);
    if(currentAlt <= initAlt + altRange){
      checkAlt1 = ps.readPressureMillibars();
      checkAlt1 = ps.pressureToAltitudeMeters(checkAlt1); 
      delay(2000);
      checkAlt2 = ps.readPressureMillibars();
      checkAlt2 = ps.pressureToAltitudeMeters(checkAlt2); 
      if(checkAlt1 - altRange < checkAlt2 < checkAlt1 + altRange){
        landed = 1;
        Serial.println("Landed");
      }
    }
  }
}
