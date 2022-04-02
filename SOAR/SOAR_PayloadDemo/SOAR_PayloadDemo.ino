/*-------------------------------------------------------------
  SOAR code for payload demo flight
  Devin Spivey and Fowler Askew
  AURA Payload 2021-22

  Program Flow:
    Receive manual commands until ARM command is received
    Once armed, ignore manual commands unless DISARM is received
    Continually transmit command/telemetry packets

    Monitor altitude sensor and run landing detection algorithm - wait until landing is detected
    Update packet to activate EASE
    Begin orientation using accelerometer
    Wait until EASE transmits that payload is ejected
    Turn on front legs on low power for a couple seconds
    Turn on back legs on low power for a couple seconds
    Turn on SLS motor until limit switch is pressed
    Update packet to indicate that MARCO/POLO can begin locating each other
    Return to full manual control mode
    
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

#define LEGS_DIR_1 1
#define LEGS_PWM_1 2
#define LEGS_DIR_2 3
#define LEGS_PWM_2 4

// RFM97 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);  // sensor objects
Adafruit_BMP280 bmp; // BMP connected I2C

float initAlt = 0;  // stores altitude at ground level
float currentAlt = 10000;
float checkAlt1 = 10000;
float checkAlt2 = 10000;
int launchThresh = 500;  // rocket must pass this altitude for landing detection to proceed
int altRange = 2; //Amount of meters (+ or -) the rocket can be above/below the starting value.
float noiseLimit = 0.1;  // amount of noise allowed between values after landing - lower will make it trigger when the rocket is more still
bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;
bool easeActive = false;
bool levelingActive = false;
bool isEjected = false;


// transmit variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 2500;  // milliseconds between tranmissions

// receive array
byte RXarray[8];  // stores received array

// radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int controls[] = {0, 0, 0, 0}; // stores arm flag, soar, sls, legs

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
  
  accel.setRange(ADXL345_RANGE_2_G);
  
  delay(250);

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
  
  initAlt = bmp.readAltitude(LOCALPRESSURE);
  Serial.println(initAlt);
  Serial.println("Startup complete");
  delay(1000);
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  digitalWrite(LED_2, LOW);
}

void loop()
{
  currentAlt = bmp.readAltitude(LOCALPRESSURE) - initAlt;  // make the altitude reading relative to ground level
  
  if(controls[0])  // if the system is armed (autonomous control)
  {
    if(!isLaunched && currentAlt > launchThresh)
    {
      isLaunched = 1;
      Serial.println("Launched");
    }
      
    if(isLaunched && !isLanded)
    {
      if(currentAlt <= altRange && currentAlt >= -altRange)
      {
        checkAlt1 = bmp.readAltitude(LOCALPRESSURE) - initAlt;
        delay(250);
        checkAlt2 = bmp.readAltitude(LOCALPRESSURE) - initAlt;
        if(checkAlt1 - noiseLimit < checkAlt2 < checkAlt1 + noiseLimit)
        {
          isLanded = 1;
          Serial.println("Landed");
        }
      }
    }

    if(isLanded && !isEjected)
    {
      
    }
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
      delay(1);
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
  receiveState = radio.readData(RXarray, 8);  // read received data to array

  if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
  {
//    Serial.println(F("[RFM97] Received packet!"));
//
//    Serial.print(F("[RFM97] Data:\t\t"));  // print data
//    Serial.print(RXarray[0]);
//    Serial.print("\t");
//    Serial.print(RXarray[1], BIN);
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
//    Serial.print("\t");
//    Serial.print(RXarray[7]);
//    Serial.print("\t");
//    Serial.println(RXarray[8]);
//    
//    Serial.print(F("\t[RFM97] RSSI: "));  // print RSSI if desired
//    Serial.print(radio.getRSSI());
//    Serial.println(F(" dBm"));

    if(RXarray[0] == 0)  // if the values are from MARCO, update the stuff
    {
      controls[0] = RXarray[1] & 0b00000001;  // controls[0] is set to the state of the arm bit
      
      controls[1] = RXarray[3];  // SOAR motor speed from RXarray
      if(~RXarray[1] & 0b00000100)  // if SOAR direction bit is not set
        controls[1] *= -1;
      
      controls[2] = RXarray[4];  // SLS speed from RXarray
      if(~RXarray[1] & 0b00001000)  // if SLS direction bit is not set
        controls[2] *= -1;
      
      controls[3] = RXarray[5];  // LEGS speed from RXarray
      if(~RXarray[1] & 0b00010000)  // if LEGS direction bit is not set
        controls[3] *= -1;
    }

    if(RXarray[0] = 1)  // if values are from EASE
    {
      
    }

    if(RXarray[0] = 3)  // if values are from POLO
    {
      
    }
  }
}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 2;  // set SOAR's system address
  if(isLanded)
    RXarray[7] = 2;
  else if(isLaunched)
    RXarray[7] = 1;
  else
    RXarray[7] = 0;
    
  if(easeActive)
    RXarray[1] |= 0b00000010;  // set EASE bit to tell EASE to activate
  
  transmitFlag = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
//  Serial.println(F("[RFM97] Sending array ... "));
  transmitState = radio.startTransmit(RXarray, 8);  // transmit array
}

void setMotors()
{
  if(controls[1] > 0)  // speed is positive
    digitalWrite(DIR_A, HIGH);
  else
    digitalWrite(DIR_A, LOW);

  if(controls[2] > 0)  // speed is positive
    digitalWrite(DIR_B, HIGH);
  else
    digitalWrite(DIR_B, LOW);

  if(controls[3] > 0)  // speed is positive
  {
    digitalWrite(LEGS_DIR_1, HIGH);
    digitalWrite(LEGS_DIR_2, HIGH);
  }
  else
  {
    digitalWrite(LEGS_DIR_1, LOW);
    digitalWrite(LEGS_DIR_2, LOW);
  }
    
  analogWrite(PWM_A, abs(controls[1]));
  analogWrite(PWM_B, abs(controls[2]));
  analogWrite(LEGS_PWM_1, abs(controls[3]));
  analogWrite(LEGS_PWM_2, abs(controls[3]));
}
