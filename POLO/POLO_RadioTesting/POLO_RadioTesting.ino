/*-------------------------------------------------------------
  POLO code for payload demo flight
  Devin Spivey and Fowler Askew
  AURA Payload 2021-22

  Program Flow:
    Receive manual commands until ARM command is received
    ONE COMMAND NEEDS TO BE FOR SUNDIAL CALIBRATION
    Store calibration in EEPROM?
    Once armed, ignore manual commands unless DISARM is received
    Continually receive and monitor packets from SOAR

    Wait until packet indicates that payload is fully deployed
    Delay a few seconds to make sure MARCO has started sending pulses
    Slowly rotate clockwise in 5 degree? increments - waiting at each stop until message is received
    Store number of encoder pulses and RSSI at each stop
    Once rotation is completed, return to location with highest RSSI
    Slowly rotate 5 degrees in each direction, noting encoder pulse count and RSSI
    Point in direction of highest RSSI and zero encoder count
    Transmit message indicating rangefinding can begin
    Store micros() once radio message is received, wait until signal is detected on analog input and log micros() again
    Calculate difference and multiply by distance coefficient
    Repeat several times and average measurements
    Determine which way to rotate for sundial time
    Rotate until necessary sensor hits halfway between max and min
    Convert encoder count to number of degrees
    Rotate back to face MARCO
    Calculate distance from MARCO in X and Y using distance and angle
    Add X and Y to MARCO location on gridded map
    Divide by box width to get box number
    Transmit distance and bearing, X and Y, and box number to MARCO
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1028.1    // used to calculate altitude (in millibar)
#define LED_2 22
#define LED_1 23
#define LIMIT_1 20
#define LIMIT_2 21

#include <RadioLib.h>  // include radio library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// motor pins
#define PWM_A 6  // PWM pin, motor A (SOAR)
#define DIR_A 5  // Direction pin, motor A
#define PWM_B 8  // PWM pin, motor B (SLS)
#define DIR_B 7  // Direction pin, motor B

// RFM97 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);  // sensor objects
Adafruit_BMP280 bmp; // BMP connected I2C

float BMPcal = 0;  // initial calibration value (ground level)
float currentAlt = 0;
float Xaccel = 0;  // x acceleration value is stored here
float lastAccel = 0;  // stores previous acceleration value
float accelThreshold = 55;  // acceleration spike threshold value
float altThreshold = 25;  // altitude threshold considered "on the ground"
unsigned int launchTime = 0;  // saves the start of the launch acceleration
bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;

// transmit variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 2500;  // milliseconds between tranmissions

// receive array
byte RXarray[9];  // stores received array

// radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int controls[] = {0, 0, 0, 0}; // stores arm flag, motor values, and latch state

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  pinMode(LIMIT_1, INPUT);
  pinMode(LIMIT_2, INPUT);

  digitalWrite(LED_2, HIGH);

  Serial.begin(115200);
  Wire.begin();
  delay(250);

  // ----- BMP280 ALTIMETER SETUP -----
  if (!bmp.begin())
    Serial.println("BMP280 initialization failed");
  else
    Serial.println("BMP280 initialized");

  unsigned int bmpStart = millis();

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_500);

  // ----- ADXL345 ACCELEROMETER SETUP -----
  if(!accel.begin())
    Serial.println("ADXL345 initialization failed");
  else
    Serial.println("ADXL345 initialized");
  
  accel.setRange(ADXL345_RANGE_16_G);
  
  delay(500);

  // ----- BEGIN RADIO SETUP -----
  // initialize RFM97 with all settings listed
  Serial.print(F("[RFM97] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                             125.0,  // bandwidth (kHz)
                             9,  // spreading factor
                             7,  // coding rate
                             0x12,  // sync word
                             17,  // output power (dBm)
                             8,  // preamble length (symbols)
                             0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check
  delay(500);

  if (receiveState == RADIOLIB_ERR_NONE)  // if radio initialized correctly
  {
    Serial.println(F("init success!"));
    for(int l=0; l < 3; l++)
    {
      digitalWrite(LED_1, HIGH);
      delay(100);
      digitalWrite(LED_1, LOW);
      delay(100);
    }
  }
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
  }

  radio.setDio0Action(setFlag);  // function that will be called when something is done
  // ----- END RADIO SETUP -----

  setMotors();  // sets the motors based on controls array

  while(bmpStart + 30000 > millis())  // wait until the BMP has been on for 30s
    delay(10);
  
  BMPcal = bmp.readAltitude(LOCALPRESSURE);
  Serial.println(BMPcal);
  Serial.println("Startup complete");
  delay(1000);
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  digitalWrite(LED_2, LOW);
}

void loop()
{
  lastAccel = Xaccel;
  sensors_event_t event;  // get ADXL345 data
  accel.getEvent(&event);
  Xaccel = abs(event.acceleration.x);  // store acceleration
  //Serial.println(Xaccel);

  if(Xaccel > accelThreshold)
  {
    if(!isLaunched)
    {
      if((lastAccel > accelThreshold))
      {
        if(launchTime + 2500 < millis())
          isLaunched = true;
      }
      else
        launchTime = millis();
    }
    else if(!isLanded)
    {
      currentAlt = bmp.readAltitude(LOCALPRESSURE) - BMPcal;
      if(abs(currentAlt) < 20)
        isLanded = true;
    }
    else
      digitalWrite(LED_2, HIGH);
  }

  if(operationDone)  // if the last operation is finished
  {
    digitalWrite(LED_1, LOW);  // No LED in between modes
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      transmitFlag = false;  // not transmitting this time
      txComplete = true;
      receiveState = radio.startReceive();  // start receiving again
      digitalWrite(LED_1, HIGH);  // LED 1 on while receive mode is active
    }

    else  // last action was receive
    {
      handleReceive();  // this stores received data to RXarray and saves RSSI
      setMotors();  // sets the motors based on controls array
      delay(10);
      transmitData();  // send a message back to GS
    }
    enableInterrupt = true;  // reenable the interrupt
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
  receiveState = radio.readData(RXarray, 9);  // read received data to array

  if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
  { 
      Serial.println(F("[RFM97] Received packet!"));

      Serial.print(F("[RFM97] Data:\t\t"));  // print data
      Serial.print(RXarray[0]);
      Serial.print("\t");
      Serial.print(RXarray[1], BIN);
      Serial.print("\t");
      Serial.print(RXarray[2]);
      Serial.print("\t");
      Serial.print(RXarray[3]);
      Serial.print("\t");
      Serial.print(RXarray[4]);
      Serial.print("\t");
      Serial.print(RXarray[5]);
      Serial.print("\t");
      Serial.print(RXarray[6]);
      Serial.print("\t");
      Serial.print(RXarray[7]);
      Serial.print("\t");
      Serial.println(RXarray[8]);
      
      Serial.print(F("\t[RFM97] RSSI: "));  // print RSSI if desired
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

    if(RXarray[0] = 2){ //Only do anything if it comes from SOAR
      if(RXarray[1] & 0b00100000){
        if(RXarray[1] & 0b01000000){
          calibrateSun();
        }
        else{
          calibrateShade();
        }
      }
    }
    else if (receiveState == RADIOLIB_ERR_CRC_MISMATCH){  // packet received malformed
      Serial.println(F("[RFM97] CRC error!"));
    }
    else if (receiveState != 2) {
      Serial.println(F("Not from SOAR - ignore"));
    }
    else  // some other error
    {
      Serial.print(F("[RFM97] Failed, code "));
      Serial.println(receiveState);
    }
  }
}

void calibrateSun() {
  //calibrate sundial in sun
  Serial.println(F("Calibrating sundial in sun"));
  return;
}

void calibrateShade() {
  //calibrate sundial in shade
  Serial.println(F("Calibrating sundial in shade"));
  return;
}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 3;  // set POLO's system address
  
  transmitFlag = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
//  Serial.println(F("[RFM97] Sending array ... "));
  transmitState = radio.startTransmit(RXarray, 9);  // transmit array
}

void setMotors()
{
  if(controls[0])
  {
    if(controls[1] > 0)  // speed is positive
      digitalWrite(DIR_A, HIGH);
    else
      digitalWrite(DIR_A, LOW);

    if(controls[2] > 0)  // speed is positive
      digitalWrite(DIR_B, HIGH);
    else
      digitalWrite(DIR_B, LOW);
      
    analogWrite(PWM_A, abs(controls[1]));
    analogWrite(PWM_B, abs(controls[2]));
  }
  else
  {
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
  }
}
