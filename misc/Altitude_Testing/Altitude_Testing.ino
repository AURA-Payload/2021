#define LOCALPRESSURE 1016.8  // used to calculate altitude

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

float initAlt = 0;  // stores altitude at ground level
float currentAlt = 10000;
float checkAlt1 = 10000;
float checkAlt2 = 10000;
bool launched = 0;
bool landed = 0;
int launchThresh = 2;  // rocket must pass this altitude for landing detection to proceed
int altRange = 1; //Amount of meters (+ or -) the rocket can be above/below the starting value.
float noiseLimit = 0.1;  // amount of noise allowed between values after landing - lower will make it trigger when the rocket is more still

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
  
  initAlt = bmp.readAltitude(LOCALPRESSURE);  // calibrate ground level altitude

  Serial.print("Altitude initialized at: ");
  Serial.println(initAlt);
  
  Serial.println("Startup complete");
}

void loop() {
  // Check current altitude. If current altitude is close enough to intial value, then enter landing detection.
  // Get altitude, wait 2 seconds, get altitude again, and compare. If the 2 values are close, we have probably landed. 
  currentAlt = bmp.readAltitude(LOCALPRESSURE) - initAlt;  // make the altitude reading relative to ground level
  if(!launched && currentAlt > launchThresh)
  {
    launched = 1;
    Serial.println("Launched");
  }
    
  if(launched && !landed)
  {
    if(currentAlt <= altRange && currentAlt >= -altRange)
    {
      checkAlt1 = bmp.readAltitude(LOCALPRESSURE) - initAlt;
      delay(250);
      checkAlt2 = bmp.readAltitude(LOCALPRESSURE) - initAlt;
      if(checkAlt1 - noiseLimit < checkAlt2 < checkAlt1 + noiseLimit)
      {
        landed = 1;
        Serial.println("Landed");
      }
    }
  }
  
  Serial.println(currentAlt);
  delay(500);
}
