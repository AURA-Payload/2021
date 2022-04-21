/*---------------------------------------------------------------
  subscaleTestLogger.ino
  Written by Fowler Askew for Teensy 4.1
  AURA Payload Team 2021-22
  Basic datalogger using the SdFat library
  Includes bits and pieces from lots of code:
    ReadCsvFile and BackwardCompatibility from SdFat
    Adafruit ADXL345 library examples
    Adafruit BMP280 library examples
    Pololu LPS library examples
    Pololu LSM6 library examples
---------------------------------------------------------------*/

#include <SdFat.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdFs sd;  // sd object
FsFile datafile;  // file object

LSM6 imu;  // sensor stuff
LPS ps;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C
char imuReport[80];

unsigned int sampleTimer = 0;  // stores time of last sample
unsigned int sampleInterval = 500;  // time between samples in milliseconds
unsigned long timestamp = 0;
String dataString = "";

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(250);
  
  Serial.println("Welcome to the Subscale board! Have a wonderful day!");
  
  delay(250);
  
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized");
  delay(500);

  // ----- LSM6DS33 IMU SETUP -----
  if (!imu.init())
    Serial.println("LSM6DS33 initialization failed");
  else
    Serial.println("LSM6DS33 initialized");
    
  imu.enableDefault();  // set default options
  
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  // ODR_XL = 1000 (1.66 kHz (high performance))
  // FS_XL = 01 (+/-16 g full scale)
  // BW_XL = 00 (400 Hz) (Overridden when XL_BW_SCAL_ODR=0)
  
  imu.writeReg(LSM6::CTRL2_G, 0b10001100);
  // ODR_G = 1000 (1.66 kHz (high performance))
  // FS_G = 11 (2000 dps)
  
  delay(500);
  
  // ----- LPS25HB ALTIMETER SETUP -----
  if (!ps.init())
    Serial.println("LPS25HB initialization failed");
  else
    Serial.println("LPS25HB initialized");
    
  ps.enableDefault();
  
  delay(500);
  
  // ----- ADXL345 ACCELEROMETER SETUP -----
  if(!accel.begin())
    Serial.println("ADXL345 initialization failed");
  else
    Serial.println("ADXL345 initialized");
  
  accel.setRange(ADXL345_RANGE_16_G);
  
  delay(500);
  
  // ----- BMP280 ALTIMETER SETUP -----
  if (!bmp.begin()) {
    Serial.println("BMP280 initialization failed");
    while(1);
  }
  else
    Serial.println("BMP280 initialized");

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  delay(500);
  
  Serial.println("Startup complete");
  
  datafile = sd.open("datalog.csv", FILE_WRITE);
  if(datafile)
  {
    datafile.println();
    datafile.print("Timestamp,IMUaccelX,IMUaccelY,IMUaccelZ,IMUgyroX,IMUgyroY,IMUgyroZ,LPSpressure,LPSaltitude,LPStemperature,ADXLaccelX,ADXLaccelY,ADXLaccelZ,BMPpressure,BMPaltitude,BMPtemperature\n");;
    datafile.close();
    Serial.println("Header written");
  }
  else
    Serial.println("error opening datalog.csv");
  //Serial.print("Timestamp,IMUaccelX,IMUaccelY,IMUaccelZ,IMUgyroX,IMUgyroY,IMUgyroZ,LPSpressure,LPSaltitude,LPStemperature,ADXLaccelX,ADXLaccelY,ADXLaccelZ,BMPpressure,BMPaltitude,BMPtemperature\n");
  delay(5000);
}

// Loop
void loop()
{
  //delay(500);
  timestamp = micros();
  dataString = "";
  
  imu.read();  // get LSM6DS33 data
  
  float LPSpress = ps.readPressureMillibars();  // get LPS25HB data
  float LPSalt = ps.pressureToAltitudeMeters(LPSpress);
  float LPStemp = ps.readTemperatureC();
  
  sensors_event_t event;  // get ADXL345 data
  accel.getEvent(&event);
  
  float BMPpress = bmp.readPressure();
  float BMPalt = bmp.readAltitude(1022.4);  // local sea level pressure in mb/hPa
  float BMPtemp = bmp.readTemperature();

  dataString += String(timestamp);
  dataString += ",";
  dataString += String(imu.a.x);
  dataString += ",";
  dataString += String(imu.a.y);
  dataString += ",";
  dataString += String(imu.a.z);
  dataString += ",";
  dataString += String(imu.g.x);
  dataString += ",";
  dataString += String(imu.g.y);
  dataString += ",";
  dataString += String(imu.g.z);
  dataString += ",";
  dataString += String(LPSpress);
  dataString += ",";
  dataString += String(LPSalt);
  dataString += ",";
  dataString += String(LPStemp);
  dataString += ",";
  dataString += String(event.acceleration.x);
  dataString += ",";
  dataString += String(event.acceleration.y);
  dataString += ",";
  dataString += String(event.acceleration.z);
  dataString += ",";
  dataString += String(BMPpress);
  dataString += ",";
  dataString += String(BMPalt);
  dataString += ",";
  dataString += String(BMPtemp);
  dataString += "\n";

  //Serial.print(dataString);

  /*datafile = sd.open("datalog.csv", FILE_WRITE);
  if(datafile)
  {
    datafile.print(dataString);
    datafile.close();
    Serial.println("data written");
  }
  else
    Serial.println("error opening datalog.csv");*/
  unsigned long looptime = micros() - timestamp;
  Serial.println(looptime);
  delay(500);
}
