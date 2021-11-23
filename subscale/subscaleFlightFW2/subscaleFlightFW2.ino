/*-------------------------------------------------------------
    Subscale flight code
    Written by Fowler Askew
    AURA Payload 2021-22
    Receives control commands from ground station
    Activates motors when told to
    Transmits telemetry to team anytime command is received
    Stores sensor data to SD card when activated

    Incoming data packet structure (from MARCO):
    [SLS direction, SLS speed,
    SOAR direction, SOAR speed,
    EASE direction, EASE speed,
    datalogging toggle]

    Outgoing data string structure (to MARCO):
    "motor 1 info, motor 2 info, motor 3 info,
    datalogging active, LPS altitude, RSSI at rocket"
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1016.8  // used to calculate altitude

#include <RadioLib.h>  // include radio library
#include <SdFat.h>  // include SD library
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// motor pins
#define PWM_A 37  // PWM pin, motor A
#define DIR_A 38  // Direction pin, motor A
#define PWM_B 36  // PWM pin, motor B
#define DIR_B 35  // Direction pin, motor B
#define PWM_C 33  // PWM pin, motor C
#define DIR_C 34  // Direction pin, motor C

// RFM95 connections:
#define CSPIN 10
#define DIO0PIN 39
#define NRSTPIN 31
#define DIO1PIN 32

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

SdFs sd;  // SD and File objects
FsFile datafile;

LSM6 imu;  // sensor stuff
LPS ps;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C

float BMPcal = 0;
float LPScal = 0;

// transmit variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 2500;  // milliseconds between tranmissions

// receive array
byte RXarray[7];  // stores received array

// radio variables
int transmitState = ERR_NONE;  // saves radio state when transmitting
int receiveState = ERR_NONE;  // saves radio state when receiving
volatile bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int controls[] = {0,0,0,0};  // stores motor values and datalog flag

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(250);
  
  // ----- BEGIN RADIO SETUP -----
  // initialize RFM95 with all settings listed
  Serial.print(F("[RFM95] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                              125.0,  // bandwidth (kHz)
                              9,  // spreading factor
                              7,  // coding rate
                              0x12,  // sync word
                              20,  // output power (dBm)
                              8,  // preamble length (symbols)
                              0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check
  delay(500);
  
  if (receiveState == ERR_NONE)  // if radio initialized correctly
    Serial.println(F("init success!"));
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
  }
  
  radio.setDio0Action(setFlag);  // function that will be called when something is done
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  // ----- END RADIO SETUP -----
  
  // ----- BEGIN SD SETUP -----
  if (!sd.begin(SdioConfig(FIFO_SDIO)))
    Serial.println("Card failed, or not present");
  
  Serial.println("card initialized");
  delay(500);
  // ----- END SD SETUP -----
  
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
  
  // ----- LPS25HB ALTIMETER SETUP -----
  if (!ps.init())
    Serial.println("LPS25HB initialization failed");
  else
    Serial.println("LPS25HB initialized");
    
  ps.enableDefault();
  
  delay(500);
  LPScal = ps.readPressureMillibars();
  LPScal = ps.pressureToAltitudeMeters(LPScal);
  
  
  // ----- ADXL345 ACCELEROMETER SETUP -----
  if(!accel.begin())
    Serial.println("ADXL345 initialization failed");
  else
    Serial.println("ADXL345 initialized");
  
  accel.setRange(ADXL345_RANGE_16_G);
  
  delay(500);
  
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
  
  delay(500);
  BMPcal = bmp.readAltitude(LOCALPRESSURE);
  
  Serial.println("Startup complete");
  
  setMotors();  // sets the motors based on controls array
  
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

void loop()
{
  if(operationDone)  // if the last operation is finished
  {
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
//      if (transmitState == ERR_NONE)  // if transmission completed successsfully
//        Serial.println(F("transmission finished!"));
//      else  // if transmission failed
//      {
//        Serial.print(F("transmission failed, code "));
//        Serial.println(transmitState);
//      }

      transmitFlag = false;  // not transmitting this time
      txComplete = true;
      receiveState = radio.startReceive();  // start receiving again
    }

    else  // last action was receive
    {
      handleReceive();  // this stores received data to RXarray and saves RSSI
      setMotors();  // sets the motors based on controls array
      transmitData();  // send a message back to GS
    }

    enableInterrupt = true;  // reenable the interrupt
  }
  
  if(controls[3])  // if the logging flag is on
  {
    String dataString = "";
    
    imu.read();  // get LSM6DS33 data
    
    dataString += String(micros());
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
    
    float LPSpress = ps.readPressureMillibars();  // get LPS25HB data
    
    dataString += String(LPSpress);
    dataString += ",";
    dataString += String(ps.pressureToAltitudeMeters(LPSpress) - LPScal);
    dataString += ",";
    dataString += String(ps.readTemperatureC());
    dataString += ",";
    
    sensors_event_t event;  // get ADXL345 data
    accel.getEvent(&event);
    
    dataString += String(event.acceleration.x);
    dataString += ",";
    dataString += String(event.acceleration.y);
    dataString += ",";
    dataString += String(event.acceleration.z);
    dataString += ",";

    dataString += String(bmp.readPressure());
    dataString += ",";
    dataString += String(bmp.readAltitude(LOCALPRESSURE) - BMPcal);
    dataString += ",";
    dataString += String(bmp.readTemperature());
    dataString += "\n";

    //Serial.print(dataString);

    datafile = sd.open("datalog.csv", FILE_WRITE);
    if(datafile)
    {
      datafile.print(dataString);
      datafile.close();
      //Serial.println("data written");
    }
//    else
//      Serial.println("error opening datalog.csv");
  }
}

void setFlag(void)  // this function is called after complete packet transmission or reception
{
  if(!enableInterrupt)  // check if the interrupt is enabled
    return;

  operationDone = true;  // something happened, set the flag
}

void handleReceive()  // performs everything necessary when data comes in
{
  receiveState = radio.readData(RXarray, 7);  // read received data to array

  if (receiveState == ERR_NONE)  // packet received correctly
  {
//    Serial.println(F("[RFM95] Received packet!"));
//
//    Serial.print(F("[RFM95] Data:\t\t"));  // print data
//    Serial.print(RXarray[0]);
//    Serial.print("\t");
//    Serial.print(RXarray[1]);
//    Serial.print("\t");
//    Serial.print(RXarray[2]);
//    Serial.print("\t");
//    Serial.print(RXarray[3]);
//    Serial.print("\t");
//    Serial.print(RXarray[4]);
//    Serial.print("\t");
//    Serial.print(RXarray[5]);
//    Serial.print("\t");
//    Serial.print(RXarray[6]);
    
    lastRSSI = radio.getRSSI();  // get RSSI to be reported
    
//    Serial.print(F("\t[RFM95] RSSI: "));  // print RSSI if desired
//    Serial.print(lastRSSI);
//    Serial.println(F(" dBm"));
   
    if(RXarray[0])
      controls[0] = -1 * RXarray[1];
    else
      controls[0] = RXarray[1];
    
    if(RXarray[2])
      controls[1] = -1 * RXarray[3];
    else
      controls[1] = RXarray[3];
    
    if(RXarray[4])
      controls[2] = -1 * RXarray[5];
    else
      controls[2] = RXarray[5];

    controls[3] = RXarray[6];
  }
//  else if (receiveState == ERR_CRC_MISMATCH)  // packet received malformed
//    Serial.println(F("[RFM95] CRC error!"));
//  else  // some other error
//  {
//    Serial.print(F("[RFM95] Failed, code "));
//    Serial.println(receiveState);
//  }
}

void transmitData()
{
  String telemetryString = "";
  
  telemetryString += String(controls[0]);
  telemetryString += ", ";
  telemetryString += String(controls[1]);
  telemetryString += ", ";
  telemetryString += String(controls[2]);
  telemetryString += ", ";
  telemetryString += String(controls[3]);
  telemetryString += ", ";
  telemetryString += String(bmp.readAltitude(LOCALPRESSURE)-BMPcal);
  telemetryString += "m, ";
  telemetryString += String(lastRSSI);
  telemetryString += "dBm\n";
  
  transmitFlag = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
  //Serial.println(F("[RFM95] Sending string ... "));
  transmitState = radio.startTransmit(telemetryString);  // transmit array 1
}

void setMotors()
{
  if(controls[0] > 0)  // speed is positive
    digitalWrite(DIR_A, HIGH);
  else
    digitalWrite(DIR_A, LOW);
    
  analogWrite(PWM_A, abs(controls[0]));
  
  if(controls[1] > 0)  // speed is positive
    digitalWrite(DIR_B, HIGH);
  else
    digitalWrite(DIR_B, LOW);
    
  analogWrite(PWM_B, abs(controls[1]));
  
  if(controls[2] > 0)  // speed is positive
    digitalWrite(DIR_C, HIGH);
  else
    digitalWrite(DIR_C, LOW);
    
  analogWrite(PWM_C, abs(controls[2]));
}
