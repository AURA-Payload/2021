/*-------------------------------------------------------------
    Subscale flight code
    Written by Fowler Askew
    AURA Payload 2021-22
    Receives control commands from ground station
    Activates motors when told to
    transmits telemetry to team

    Incoming data packet structure (from MARCO):
    [motor enable, nudge mode,
    SLS command, SOAR command,
    EASE command, datalogging toggle]

    Outgoing data packet structure (to MARCO):
    [motor 1 info, motor 2 info,
    motor 3 info, datalogging active]

    Motor numbers: 0 - CW, 255 - CCW, else 0

    Datalog format:
    timestamp,IMUxaccel,IMUyaccel,IMUzaccel,IMUxgyr,IMUygyr,IMUzgyr,LPSpress,LPSalt,ADXLx,ADXLy,ADXLz,BMPpress,BMPalt
  -------------------------------------------------------------*/
#include <RadioLib.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// set up motor pins
#define PWM_A 37  // PWM pin, motor A
#define DIR_A 38  // Direction pin, motor A
#define PWM_B 36  // PWM pin, motor B
#define DIR_B 35  // Direction pin, motor B
#define PWM_C 33  // PWM pin, motor C
#define DIR_C 34  // Direction pin, motor C

// set up radio parameters
#define radioCarrier 915.0  // carrier frequency
#define radioBW 125.0  // bandwidth (kHz)
#define radioSF 9  // spreading factor
#define radioCR 7  // coding rate
#define radioSync 0x12  // sync word (same for TX and RX)
#define radioPower 15  // output power (dBm)
#define radioPreamble 8  // preamble length (symbols)
#define radioGain 0  // gain (0 is automatic control)
#define radioCRC true  // cyclic redundancy check

#define pin_cs 10  // radio chip select
#define pin_nrst 31  // radio reset
#define pin_dio0 39  // radio DIO0
#define pin_dio1 32  // radio DIO1

RFM95 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);  // create radio object

LSM6 imu;  // sensor stuff
LPS ps;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C
char imuReport[80];

byte commands[] = {0x0, 0x0, 0x7F, 0x7F, 0x7F, 0x0};  // create command array
byte telemetry[] = {0x0, 0x0, 0x0, 0x0};  // telemetry array

// RX interrupt setup
volatile bool receivedFlag = false;  // received flag
volatile bool enableInterrupt = true;  // is it allowed

// flags for loop events
bool newCommands = false;  // are there new commands received
bool datalogActive = false;
unsigned int sampleTimer = 0;  // stores time of last sample
unsigned int sampleInterval = 5;  // time between samples in milliseconds
unsigned int telemetryTimer = 0;  // stores time of last transmission
unsigned int telemetryInterval1 = 15000;  // time between transmissions, datalogging inactive
unsigned int telemetryInterval2 = 1000;  // time between transmissions, datalogging active


void setFlag(void) {  // sets received flag
  if(!enableInterrupt)  // is it allowed
    return;

  receivedFlag = true;  // if there's a packet
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(250);
  
  Serial.println("Welcome to the Subscale board! Have a wonderful day!");

  if(imu.init())
    Serial.println("IMU initialized");
  // 0x84 = 0b10000100
  // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 01 (+/-16 g full scale)
  imu.writeReg(LSM6::CTRL1_XL, 0x84);
  // 0x88 = 0b10001000
  // ODR = 1000 (1.66 kHz (high performance)); FS_G = 10 (1000 dps)
  imu.writeReg(LSM6::CTRL2_G, 0x88);

  if(ps.init())
    Serial.println("Pressure initialized");
  ps.enableDefault();

  if(accel.begin())
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL initialized");

//  if(bmp.begin())
//    Serial.println("BMP initialized");
//  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
//                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
//                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
//                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
//                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.print(F("[RFM95] Initializing ... "));  // initialize radio
  int state = radio.begin(radioCarrier, radioBW, radioSF, radioCR, radioSync, radioPower, radioPreamble, radioGain);

  if (state == ERR_NONE) {  // report if radio started correctly
    Serial.println(F("Radio boot success!"));
  } else {
    Serial.print(F("Radio boot failed, code "));
    Serial.println(state);
    Serial.println("Try rebooting");
    while (true);
  }

  radio.setCRC(radioCRC);  // set up CRC
  radio.setDio0Action(setFlag);  // set the function that will be called
  beginListen();
}

// Loop
void loop()
{
  
  if(receivedFlag)
  {
    Serial.println("received flag set");
    getCommands();
  }

  if(newCommands)
  {
    Serial.println("running newcommand routine");
    if(commands[0])  // if motors are enabled, activate motors
    {
      if(commands[2] == 0)  // SLS
        setMotorA(-255);
      else if(commands[2] == 255)
        setMotorA(255);
      else
        setMotorA(0);
        
      if(commands[3] == 0)  // SOAR
        setMotorB(-200);
      else if(commands[3] == 255)
        setMotorB(200);
      else
        setMotorB(0);
        
      if(commands[4] == 0)  // EASE
        setMotorC(-255);
      else if(commands[4] == 255)
        setMotorC(255);
      else
        setMotorC(0);
    }
    else  // if motors disabled, stop motors
    {
      setMotorA(0);
      setMotorB(0);
      setMotorC(0);
    }
    
    if(commands[5])    // if datalogging command is true
    {
      Serial.println("activating logging");
      datalogActive = true;
      telemetry[3] = 0xFF;
    }
    else
    {
      Serial.println("deactivating logging");
      datalogActive = false;
      telemetry[3] = 0x00;
    }
    
    newCommands = false;  // reset newCommands flag
    Serial.println("exiting newcommands");
  }
  
  if(datalogActive)
  {
    if((sampleTimer + sampleInterval) < millis())
    {
      sampleTimer = millis();
      //logData();
    }
    
//    if((telemetryTimer + telemetryInterval2) < millis())
//    {
//      telemetryTimer = millis();
//      Serial.println("telemetry");
//      //transmitTelemetry();
//    }
  }
//  else
//  {
//    if((telemetryTimer + telemetryInterval1) < millis())
//    {
//      telemetryTimer = millis();
//      Serial.println("telemetry");
//      //transmitTelemetry();
//    }
//  }
}

//void transmitTelemetry()
//{
//  int state = radio.transmit(telemetry, 4);
//
//  if (state == ERR_NONE) {
//    // the packet was successfully transmitted
//    Serial.println(F("Transmitted!"));
//
//  } else if (state == ERR_PACKET_TOO_LONG) {
//    // the supplied packet was longer than 256 bytes
//    //Serial.println(F("Packet too long"));
//
//  } else if (state == ERR_TX_TIMEOUT) {
//    // timeout occurred while transmitting packet
//    Serial.println(F("TX timeout"));
//
//  } else {
//    // some other error occurred
//    Serial.print(F("TX failed, code "));
//    Serial.println(state);
//  }
//
//  beginListen();
//}

void beginListen()
{
  volatile int state = radio.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("Now listening"));
  } else {
    Serial.print(F("RX failed, code "));
    Serial.println(state);
  }
}

void getCommands()  // gets the received data
{
  enableInterrupt = false;  // disable ISR
  receivedFlag = false;  // reset flag
  volatile byte newCommandArray[6];
  volatile int rxState = radio.readData(newCommandArray, 6);  // get data

  Serial.print(newCommandArray[0]);
  Serial.print("\t");
  Serial.print(newCommandArray[1]);
  Serial.print("\t");
  Serial.print(newCommandArray[2]);
  Serial.print("\t");
  Serial.print(newCommandArray[3]);
  Serial.print("\t");
  Serial.print(newCommandArray[4]);
  Serial.print("\t");
  Serial.println(newCommandArray[5]);

  if(rxState == ERR_NONE)
  {
    Serial.println("Data received correctly");
    commands[0] = newCommandArray[0];
    commands[1] = newCommandArray[1];
    commands[2] = newCommandArray[2];
    commands[3] = newCommandArray[3];
    commands[4] = newCommandArray[4];
    commands[5] = newCommandArray[5];
    newCommands = true;
  }
  else
  {
    Serial.println("Data received erroneously");
    newCommands = false;
  }

  beginListen();
  enableInterrupt = true;  // enable ISR
}

void logData()  // collects sensor data & stores
{
 Serial.println("datalog");
     
 unsigned long timestamp = micros();
 String dataString = "";

 imu.read();
 snprintf(imuReport, "%6d,%6d,%6d,%6d,%6d,%6d,", imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z);

 float pressure = ps.readPressureMillibars();
 float altitude = ps.pressureToAltitudeMeters(pressure);

 sensors_event_t event; 
 accel.getEvent(&event);

//  bmp.readPressure();  
//  float bmpPressure = bmp.readPressure();
//  float bmpAlt = bmp.readAltitude(1022.4);

 dataString += String(timestamp);
 dataString += ",";
 dataString += imuReport;
 dataString += String(pressure);
 dataString += ",";
 dataString += String(altitude);
 dataString += ",";
 dataString += String(event.acceleration.x);
 dataString += ",";
 dataString += String(event.acceleration.y);
 dataString += ",";
 dataString += String(event.acceleration.z);
//  dataString += ",";
//  dataString += String(bmpPressure);
//  dataString += ",";
//  dataString += String(bmpAlt);

 Serial.println(dataString);

//  File dataFile = SD.open("datalog.txt", FILE_WRITE);
//  if(dataFile)
//  {
//    dataFile.println(dataString);
//    dataFile.close();
//  }
}

void setMotorA(int motorSpeed)
{
  //------------------------------------
  // Accepts an input from -255 to 255
  //------------------------------------
  if(motorSpeed > 0)
    telemetry[0] = 255;
  else if (motorSpeed < 0)
    telemetry[0] = 0;
  else
    telemetry[0] = 127;
  
  if(motorSpeed > 0)  // motorSpeed is positive
    digitalWrite(DIR_A, HIGH);
    
  else
    digitalWrite(DIR_A, LOW);
    
  analogWrite(PWM_A, abs(motorSpeed));
  Serial.print("Motor A set to: ");
  Serial.println(motorSpeed);
}

void setMotorB(int motorSpeed)
{
  //------------------------------------
  // Accepts an input from -255 to 255
  //------------------------------------
  if(motorSpeed > 0)
    telemetry[1] = 255;
  else if (motorSpeed < 0)
    telemetry[1] = 0;
  else
    telemetry[1] = 127;
  
  if(motorSpeed > 0)  // motorSpeed is positive
    digitalWrite(DIR_B, HIGH);
    
  else
    digitalWrite(DIR_B, LOW);
    
  analogWrite(PWM_B, abs(motorSpeed));
  Serial.print("Motor B set to: ");
  Serial.println(motorSpeed);
}

void setMotorC(int motorSpeed)
{
  //------------------------------------
  // Accepts an input from -255 to 255
  //------------------------------------
  if(motorSpeed > 0)
    telemetry[2] = 255;
  else if (motorSpeed < 0)
    telemetry[2] = 0;
  else
    telemetry[2] = 127;
  
  if(motorSpeed > 0)  // motorSpeed is positive
    digitalWrite(DIR_C, HIGH);
    
  else
    digitalWrite(DIR_C, LOW);
    
  analogWrite(PWM_C, abs(motorSpeed));
  Serial.print("Motor C set to: ");
  Serial.println(motorSpeed);
}
