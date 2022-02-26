/*-------------------------------------------------------------
    SOAR code for first flight
    Devin Spivey
    AURA Payload 2021-22
  "
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1028.1    // used to calculate altitude (in millibar)
#define LED_1 22
#define LED_2 23
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

  Serial.begin(115200);
  Wire.begin();
  delay(250);

  // ----- BMP280 ALTIMETER SETUP -----
  if (!bmp.begin())
    Serial.println("BMP280 initialization failed");
  else
    Serial.println("BMP280 initialized");

  int bmpStart = millis();

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
}

void loop()
{

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
   
    controls[0] = RXarray[1] & 0b00000001;  // controls[0] is set to the state of the arm bit

    controls[1] = RXarray[3];  // SOAR motor speed from RXarray
    if(~RXarray[1] & 0b00000100)  // if SOAR direction bit is not set
      controls[1] *= -1;

    controls[2] = RXarray[4];  // SLS speed from RXarray
    if(~RXarray[2] & 0b00001000)  // if SLS direction bit is not set
      controls[2] *= -1;
      
  }
//  else if (receiveState == RADIOLIB_ERR_CRC_MISMATCH)  // packet received malformed
//    Serial.println(F("[RFM97] CRC error!"));
//  else  // some other error
//  {
//    Serial.print(F("[RFM97] Failed, code "));
//    Serial.println(receiveState);
//  }
}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 2;  // set SOAR's system address
  
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
