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

#define transmitDelay 10  // how many milliseconds to wait before transmitting stuff
#define transmitInterval 500  // milliseconds between transmissions

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);  // sensor objects
Adafruit_BMP280 bmp; // BMP connected I2C

unsigned long altTimer = 0;  // stores timestamps of altitude samples
unsigned long altInterval = 250;
float initAlt = 0;  // stores altitude at ground level
float checkAlt1 = 0;  // stores the previous altitude sample
float checkAlt2 = 0;  // stores the current altitude sample
int launchThresh = 609;  // rocket must pass this altitude (meters) for landing detection to proceed
int altRange = 10; //  Amount of meters (+ or -) the rocket can be above/below the starting value.
float noiseLimit = 0.1;  // amount of noise allowed between values after landing - lower will make it trigger when the rocket is more still
bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;

// Leveling varibles
unsigned long accelTimer = 0;  // stores timestamp of levelling updates
unsigned long accelInterval = 10;  // time between levelling updates
float levelValue = 1.18;  // target value for level
float levelTolerance = 0.1;  // acceptable range for "level"
float errorTerm = 0;
float pGain = 50;
bool isLevel = false; //flag for when leveling is done

unsigned long legsTimer = 0;
unsigned long legsDuration = 3000;  // how long to deploy the legs
int legsPower = 30;  // how much power to send to the legs

int slsPower = 255;

// Deployment variables
bool easeActivated = false;
bool easeDeployed = false;
bool legsActivated = false;
bool legsDeployed = false;
bool slsActivated = false;
bool slsDeployed = false;
bool fullyDeployed = false;
bool rangeFinding = false;

// transmit variables
unsigned long transmitTimer = 0;  // stores the time of the last transmission

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
unsigned long receiveTime = 0;  // stores the time when a packet was received
bool hasTransmitted = true;

// motor speed control variables
int armVar = 0;
int soarVar = 0;
int slsVar = 0;
int legsVar = 0;

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(LEGS_PWM_1, OUTPUT);
  pinMode(LEGS_DIR_1, OUTPUT);
  pinMode(LEGS_PWM_2, OUTPUT);
  pinMode(LEGS_DIR_2, OUTPUT);

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
  
  delay(250);

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

  setMotors();  // sets the motors based on vars

  while(bmpStart + 20000 > millis())  // wait until the BMP has been on for 20s
    delay(10);
  
  initAlt = bmp.readAltitude(LOCALPRESSURE);
  checkAlt2 = initAlt;
  Serial.println(initAlt);
  Serial.println("Startup complete");
  delay(1000);
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  digitalWrite(LED_2, LOW);

  //TESTING VARIABLES = REMOVE!!!
  //armVar = 1;
  //isLaunched = true;
  //isLanded = true;
  
}

void loop()
{
  if(armVar) //autonomous control section
  {  
    if(!isLaunched && (bmp.readAltitude(LOCALPRESSURE) - initAlt) > launchThresh)  // if current altitude is greater than threshold
    {
      soarVar = 0;  // reset motors
      slsVar = 0;
      legsVar = 0;
      
      Serial.println(bmp.readAltitude(LOCALPRESSURE) - initAlt);  // print altitude
      isLaunched = true;
      Serial.println("Launched");
    }
      
    if(isLaunched && !isLanded && millis()-altTimer >= altInterval)  // sample new values every 250ms and check if we're landed
    {
      soarVar = 0;  // reset motors
      slsVar = 0;
      legsVar = 0;
      
      checkAlt1 = checkAlt2;  // transfer last sample to checkAlt1
      checkAlt2 = bmp.readAltitude(LOCALPRESSURE) - initAlt;  // sample a new altitude
      if(-altRange <= checkAlt2 <= altRange)  // if we are within the range
      {
        if(checkAlt1 - noiseLimit < checkAlt2 < checkAlt1 + noiseLimit)
        {
          isLanded = true;
          Serial.println("Landed");
        }
      }
    }

    if(isLanded && !easeActivated){  // activate EASE
      soarVar = 0;  // reset motors
      slsVar = 0;
      legsVar = 0;
      
      easeActivated = true;
      Serial.println("Ease set to activate");
    }
  
    if(easeDeployed && !isLevel && millis()-accelTimer >= accelInterval){  // run levelling code on an interval
      slsVar = 0;  // reset motors
      legsVar = 0;
      
      sensors_event_t event;
      accel.getEvent(&event);
      //Display the results (acceleration is measured in m/s^2)
//      Serial.print("Y: ");
//      Serial.print(event.acceleration.y);
//      Serial.print(" m/s^2\t");
    
      errorTerm = levelValue - event.acceleration.y;
      if(event.acceleration.z < 0){
        errorTerm *= 1000;
      }
      soarVar = errorTerm * pGain;
      soarVar = constrain(soarVar, -255, 255);
    
      Serial.print("SOAR motor value: ");
      Serial.println(soarVar);
    
      if(abs(errorTerm) < levelTolerance){
        Serial.println("\tLevel :)");
        isLevel = true;
        soarVar = 0;
      }
      else
        Serial.println("\tNot level >:(");
    }
  
  
    if(isLevel && !legsActivated){  // wait until level and activate legs
      soarVar = 0;  // reset motors
      slsVar = 0;
      
      Serial.println("Activating legs");
      legsTimer = millis();
      legsVar = legsPower;
      legsActivated = true;
    }

    if(legsActivated && !legsDeployed && millis()-legsTimer >= legsDuration){  // if legs have been running for the duration
      soarVar = 0;  // reset motors
      slsVar = 0;
      legsVar = 0;
      
      legsDeployed = true;
      Serial.println("Legs Deployed");
    }
  
    if(legsDeployed && !slsDeployed && !digitalRead(LIMIT_1)){  // if soar limit switch is not triggered
      soarVar = 0;  // reset motors
      legsVar = 0;
      
      slsVar = slsPower;
    }
    else if(digitalRead(LIMIT_1)){  // if SOAR is extending and limit switch is pressed
      soarVar = 0;  // reset motors
      legsVar = 0;
      
      slsVar = 0;  // stop sls
      slsDeployed = true;
      Serial.println("SLS Deployed");
    }

    if(slsDeployed && !rangeFinding){  // once SLS is deployed
      soarVar = 0;  // reset motors
      slsVar = 0;
      legsVar = 0;
      
      fullyDeployed = true;  // set deployed flag
      RXarray[1] = RXarray[1] |= 0b10000000; //Set bit to let MARCO know we are ready for range finding
      rangeFinding = true;  // indicate that rangefinding is eing activated
      Serial.println("Activating Range Finding");
    }
  }

  if(operationDone)  // if the last radio operation is finished
  {
    digitalWrite(LED_1, LOW);  // No LEDs in between modes
    digitalWrite(LED_2, LOW);  // No LEDs in between modes
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      transmitFlag = false;  // not transmitting this time
      txComplete = true;
      digitalWrite(LED_1, HIGH);  // LED 1 on while receive mode is active
    }

    else{  // last action was receive
      handleReceive();  // this stores received data to RXarray and saves RSSI
    }
    Serial.println("Listening for packets");
    receiveState = radio.startReceive();  // start receiving again
    enableInterrupt = true;  // reenable the interrupt
  }

  if(/*(!hasTransmitted && millis() - receiveTime >= transmitDelay) || */millis() - transmitTimer >= transmitInterval)
  {
    transmitData();  // transmit a packet
  }

  setMotors();  // update the motors
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
    Serial.println(RXarray[7]);
    
    Serial.print(F("\t[RFM97] RSSI: "));  // print RSSI if desired
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    if(RXarray[0] == 0)  // if the values are from MARCO, update the stuff
    {
      armVar = RXarray[1] & 0b00000001;  // armVar is set to the state of the arm bit

      if(!armVar){  // if autonomous control is deactivated
        soarVar = RXarray[3];  // SOAR motor speed from RXarray
        if(~RXarray[1] & 0b00000100)  // if SOAR direction bit is not set
          soarVar *= -1;
        
        slsVar = RXarray[4];  // SLS speed from RXarray
        if(~RXarray[1] & 0b00001000)  // if SLS direction bit is not set
          slsVar *= -1;
        
        legsVar = RXarray[5];  // LEGS speed from RXarray
        if(~RXarray[1] & 0b00010000)  // if LEGS direction bit is not set
          legsVar *= -1;
      }
    }

    if(RXarray[0] == 1) //if value is from EASE, check to see if deployed
    {
      if(RXarray[2] == 1) //Some flag value that EASE can set once deployed and transmit - does not have to be 1. 
      {
        easeDeployed = true;
      }
    }
  }
}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 2;  // set SOAR's system address

  // Launch status
  if(isLanded)
    RXarray[7] = 2;
  else if(isLaunched)
    RXarray[7] = 1;
  else
    RXarray[7] = 0;


  //Ease activation
  if(easeActivated){
    RXarray[2] = 255;
  }
  
  transmitFlag = true;
  txComplete = false;
  hasTransmitted = true;
  transmitTimer = millis();  // reset transmit timer
  
  Serial.print(F("[RFM97] Sending array ... "));
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
  Serial.println(RXarray[7]);
  transmitState = radio.startTransmit(RXarray, 8);  // transmit array
}

void setMotors()
{
  if(soarVar > 0)  // speed is positive
    digitalWrite(DIR_A, HIGH);
  else
    digitalWrite(DIR_A, LOW);

  if(slsVar > 0)  // speed is positive
    digitalWrite(DIR_B, HIGH);
  else
    digitalWrite(DIR_B, LOW);

  if(legsVar > 0)  // speed is positive
  {
    digitalWrite(LEGS_DIR_1, HIGH);
    digitalWrite(LEGS_DIR_2, HIGH);
  }
  else
  {
    digitalWrite(LEGS_DIR_1, LOW);
    digitalWrite(LEGS_DIR_2, LOW);
  }
    
  analogWrite(PWM_A, abs(soarVar));
  analogWrite(PWM_B, abs(slsVar));
  analogWrite(LEGS_PWM_1, abs(legsVar));
  analogWrite(LEGS_PWM_2, abs(legsVar));
}
