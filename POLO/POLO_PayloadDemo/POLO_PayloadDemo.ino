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
#define PWM_1 5  // PWM pin, motor 1 (7 for EASE board)
#define DIR_1 4 // Direction pin, motor 1 (8 for EASE board)
#define LEDPIN 0
#define interruptPin 2 //Interrupt pin (6 for EASE board)

#include <RadioLib.h>  // include radio library

// RFM97 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

// receive array
byte RXarray[1];  // stores received array
float dataArray[2][10000];  // stores data
int i = 0;

// radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

int debounceTime = 10;  // number of microseconds to wait in the ISR for debouncing
int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 300; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 7;  // encoder pulses (rising and falling) for one rotation
volatile int pulseCount = 0;
volatile bool fullRotation = false;

void setup()
{
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  // ----- BEGIN RADIO SETUP -----
  // initialize RFM97 with all settings listed
  Serial.print(F("[RFM97] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                             125.0,  // bandwidth (kHz)
                             9,  // spreading factor
                             7,  // coding rate
                             0x12,  // sync word
                             1,  // output power (dBm)
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

  delay(10000);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(DIR_1, HIGH); //Set motor direction forward
  analogWrite(PWM_1, 25); //Turn on motor, low speed
  delayMicroseconds(2);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, RISING); //If value of interruptPin changes, run count.
}

void loop()
{
  if(operationDone)  // if the last operation is finished
  {
    dataArray[0][i] = pulseCount;
    
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag
    
    dataArray[1][i] = radio.getRSSI();

    i++;
    
    enableInterrupt = true;  // reenable the interrupt
  }
}

void setFlag(void)  // this function is called after complete packet transmission or reception
{
  if(!enableInterrupt)  // check if the interrupt is enabled
    return;

  operationDone = true;  // something happened, set the flag
}

void count()
{
  if
  pulseCount++;
  if(pulseCount >= (totalDistance * gearRatio * pulsePerRotate)) //Check to see if fully extended. If so, change value or something. 
  {
    detachInterrupt(digitalPinToInterrupt(interruptPin)); //stop interrupting (this has to happen or the code will never turn the motor off)
    analogWrite(PWM_1, 0); //Turn off motor
    digitalWrite(LEDPIN, LOW);  // turn off LED
    digitalWrite(
  }
  delayMicroseconds(debounceTime);  // waits some amount of time to avoid triggering multiple times for one encoder pulse
}
