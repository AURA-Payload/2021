/*-------------------------------------------------------------
    SOAR Board Testing
    Devin Spivey
    
    AURA Payload 2021-22
"
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1016.8  // used to calculate altitude
#define LED_1 22
#define LED_2 23
#define LIMIT_1 20
#define LIMIT_2 21

#include <RadioLib.h>  // include radio library
#include <SdFat.h>  // include SD library
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// motor pins
#define PWM_A 6  // PWM pin, motor A
#define DIR_A 5  // Direction pin, motor A
#define PWM_B 8  // PWM pin, motor B
#define DIR_B 7  // Direction pin, motor B

// RFM95 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

Servo SERVO_1;
Servo SERVO_2;
Servo SERVO_3;
Servo SERVO_4;
Servo SERVO_5;

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
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
volatile bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int controls[] = {0,0,0,0};  // stores motor values and datalog flag

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LIMIT_1, INPUT);
  pinMode(LIMIT_2, INPUT);

  SERVO_1.attach(0);
  SERVO_2.attach(1);
  SERVO_3.attach(2);
  SERVO_4.attach(3);
  SERVO_5.attach(4);

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
  
  if (receiveState == RADIOLIB_ERR_NONE)  // if radio initialized correctly
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
  //Turn motor A on forward, then reverse.
  digitalWrite(DIR_A, HIGH);
  analogWrite(PWM_A, 100);
  delay(2000);
  analogWrite(PWM_A, 0);

  digitalWrite(DIR_A, LOW);
  analogWrite(PWM_A, 100);
  delay(2000);
  analogWrite(PWM_A, 0);

  //Turn motor B on forward, then reverse.
  digitalWrite(DIR_B, HIGH);
  analogWrite(PWM_B, 100);
  delay(2000);
  analogWrite(PWM_B, 0);

  digitalWrite(DIR_B, LOW);
  analogWrite(PWM_B, 100);
  delay(2000);
  analogWrite(PWM_B, 0);
  

  sensors_event_t event;  // get ADXL345 data
  accel.getEvent(&event);


  String dataString = "";
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

  Serial.print(dataString);
  
  digitalWrite(LED_1, HIGH);

  //For 30 seconds, while LED 1 is on, if you press a limit switch, LED 2 will turn on
  int startTime = millis();
  int runningTime = millis() - startTime;
  while(runningTime < 60000) {
    if(digitalRead(LIMIT_1 == LOW)) {
      digitalWrite(LED_2, HIGH);
      delay(1000);
      digitalWrite(LED_2, LOW);
    }
    if(digitalRead(LIMIT_2 == LOW)) {
      digitalWrite(LED_2, HIGH);
      delay(1000);
      digitalWrite(LED_2, LOW);
    }
    runningTime = millis() - startTime;  
  }
  
  digitalWrite(LED_1, LOW);

  SERVO_1.write(0);
  SERVO_2.write(0);
  SERVO_3.write(0);
  SERVO_4.write(0);
  SERVO_5.write(0);
  delay(2000);
  SERVO_1.write(180);
  SERVO_2.write(180);
  SERVO_3.write(180);
  SERVO_4.write(180);
  SERVO_5.write(180);
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

  if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
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
}