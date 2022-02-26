/*-------------------------------------------------------------
    SOAR Board Testing
    Devin Spivey
    
    AURA Payload 2021-22
"
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1028.1    // used to calculate altitude (in millibar)
#define LED_1 22
#define LED_2 23
#define LIMIT_1 20
#define LIMIT_2 21

#define SERVO1PIN 0
#define SERVO2PIN 1
#define SERVO3PIN 2
#define SERVO4PIN 3
#define SERVO5PIN 4

#include <RadioLib.h>  // include radio library
#include <Wire.h>
//#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// motor pins
#define PWM_A 6  // PWM pin, motor A
#define DIR_A 5  // Direction pin, motor A
#define PWM_B 8  // PWM pin, motor B
#define DIR_B 7  // Direction pin, motor B

// RFM97 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

//Servo SERVO_1;
//Servo SERVO_2;
//Servo SERVO_3;
//Servo SERVO_4;
//Servo SERVO_5;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C

float BMPcal = 0;

// receive array
byte RXarray[9];  // stores received array

// control variables
int controls[] = {0,0,0,0};  // stores motor values and datalog flag

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LIMIT_1, INPUT);
  pinMode(LIMIT_2, INPUT);

//  SERVO_1.attach(SERVO1PIN);
//  SERVO_2.attach(SERVO2PIN);
//  SERVO_3.attach(SERVO3PIN);
//  SERVO_4.attach(SERVO4PIN);
//  SERVO_5.attach(SERVO5PIN);

  Serial.begin(115200);
  Wire.begin();
  delay(250);
  
  // ----- BEGIN RADIO SETUP -----
  // initialize RFM97 with all settings listed
  Serial.print(F("[RFM97] Initializing ... "));
  int receiveState = radio.begin(915.0,  // carrier freq (MHz)
                              125.0,  // bandwidth (kHz)
                              9,  // spreading factor
                              7,  // coding rate
                              0x12,  // sync word
                              20,  // output power (dBm)
                              8,  // preamble length (symbols)
                              0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check
  delay(500);
  
  if (receiveState == RADIOLIB_ERR_NONE)  // if radio initialized correctly
    Serial.println(F("init success!"));
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
  }
  // ----- END RADIO SETUP -----
  
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
  
  delay(30000);
  BMPcal = bmp.readAltitude(LOCALPRESSURE);
  Serial.println(BMPcal);
  
  Serial.println("Startup complete");
  
  delay(1000);
}

void loop()
{
  digitalWrite(LED_1, HIGH);
  
  //Turn motor A on forward, then reverse.
  digitalWrite(DIR_A, HIGH);
  analogWrite(PWM_A, 100);
  delay(1000);
  analogWrite(PWM_A, 0);

  delay(100);

  digitalWrite(DIR_A, LOW);
  analogWrite(PWM_A, 100);
  delay(1000);
  analogWrite(PWM_A, 0);

  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, HIGH);

  //Turn motor B on forward, then reverse.
  digitalWrite(DIR_B, HIGH);
  analogWrite(PWM_B, 100);
  delay(1000);
  analogWrite(PWM_B, 0);

  delay(100);

  digitalWrite(DIR_B, LOW);
  analogWrite(PWM_B, 100);
  delay(1000);
  analogWrite(PWM_B, 0);

  digitalWrite(LED_2, LOW);
  

  sensors_event_t event;  // get ADXL345 data
  accel.getEvent(&event);


  String dataString = "";
  dataString += String(event.acceleration.x);
  dataString += ",";
  dataString += String(event.acceleration.y);
  dataString += ",";
  dataString += String(event.acceleration.z);
  dataString += ",";
  dataString += String(bmp.readAltitude(LOCALPRESSURE) - BMPcal);
  dataString += "\n";

  Serial.print(dataString);

//  SERVO_1.write(0);
//  delay(100);
//  SERVO_2.write(0);
//  delay(100);
//  SERVO_3.write(0);
//  delay(100);
//  SERVO_4.write(0);
//  delay(100);
//  SERVO_5.write(0);
//  delay(2000);
//  SERVO_1.write(180);
//  delay(100);
//  SERVO_2.write(180);
//  delay(100);
//  SERVO_3.write(180);
//  delay(100);
//  SERVO_4.write(180);
//  delay(100);
//  SERVO_5.write(180);
//  delay(100);
}
