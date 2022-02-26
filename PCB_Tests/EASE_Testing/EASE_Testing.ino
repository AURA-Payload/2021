/*-------------------------------------------------------------
    EASE Board Testing
    Devin Spivey
    AURA Payload 2021-22
  "
  -------------------------------------------------------------*/
#define LOCALPRESSURE 1016.8  // used to calculate altitude
#define LED_1 18
#define LED_2 19
#define LIMIT_1 16
#define LIMIT_2 17

#include <RadioLib.h>  // include radio library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// motor pins
#define PWM_A 6 // PWM pin, motor A
#define DIR_A 5  // Direction pin, motor A
#define PWM_B 9  // PWM pin, motor B
#define DIR_B 8  // Direction pin, motor B

// RFM95 connections:
#define CSPIN 10
#define DIO0PIN 3
#define NRSTPIN 2
#define DIO1PIN 4

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C

float BMPcal = 0;

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
int controls[] = {0, 0, 0, 0}; // stores motor values and datalog flag

void setup()
{

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  pinMode(LIMIT_1, INPUT_PULLUP);
  pinMode(LIMIT_2, INPUT_PULLUP);

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
  
  delay(500);
  BMPcal = bmp.readAltitude(LOCALPRESSURE);

  

  Serial.println("Startup complete");

  setMotors();  // sets the motors based on controls array
}

void loop()
{

  //Turn motor A on forward, then reverse.
  digitalWrite(DIR_A, HIGH);
  analogWrite(PWM_A, 100);
  delay(2000);
  analogWrite(PWM_A, 0);

  delay(250);

  digitalWrite(DIR_A, LOW);
  analogWrite(PWM_A, 100);
  delay(2000);
  analogWrite(PWM_A, 0);

  //Turn motor B on forward, then reverse.
  digitalWrite(DIR_B, HIGH);
  analogWrite(PWM_B, 100);
  delay(2000);
  analogWrite(PWM_B, 0);

  delay(250);
  
  digitalWrite(DIR_B, LOW);
  analogWrite(PWM_B, 100);
  delay(2000);
  analogWrite(PWM_B, 0);

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
