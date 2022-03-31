#define LOCALPRESSURE 1016.8  // used to calculate altitude

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

float initAlt = 0;
float currentAlt = 10000;
float checkAlt1 = 10000;
float checkAlt2 = 10000;
int landed = 0;
int altRange = 1; //Amount of meters the rocket can be above the starting value. 

void setup() {
  // put your setup code here, to run once:
  // ----- BMP280 ALTIMETER SETUP -----
  if (!bmp.begin())
    Serial.println("BMP280 initialization failed");
  else
    Serial.println("BMP280 initialized");

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  delay(20000);
  initAlt = bmp.readAltitude(LOCALPRESSURE);
  
  Serial.println("Startup complete");
}

void loop() {
  // Check current altitude. If current altitude is close enough to intial value, then enter landing detection.
  // Get altitude, wait 2 seconds, get altitude again, and compare. If the 2 values are close, we have probably landed. 
  while(landed == 0){
    currentAlt = bmp.readAltitude(LOCALPRESSURE);
    if(currentAlt <= initAlt + altRange){
      checkAlt1 = bmp.readAltitude(LOCALPRESSURE);
      delay(2000);
      checkAlt2 = bmp.readAltitude(LOCALPRESSURE);
      if(checkAlt1 - 1 < checkAlt2 < checkAlt1 + 1){
        landed = 1;
        Serial.println("Landed");
      }
    }
  }
}
