// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM_1 5
#define DIR_1 4

volatile int posi = 0; // specify posi as volatile

int kp = 5;
int pos = 0;
int e = 0;
int u = 0;
int pwr = 0;
int dir = 1;

int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 302; // for testing output shaft accuracy: 302 for POLO, 349 for EASE
int pulsePerRotate = 7;  // encoder pulses (rising) for one rotation: 7 for POLO, 11 for EASE
int screwPitch = 1;  // TPI of leadscrew
int totalRotations = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // sets the target to hit (should be 1 shaft rotation)
int target = 0;

int targetInterval = 3000;  // 2.5 seconds between switching targets
unsigned long targetSwitch = 0;
int printerval = 10;  // millisecond interval to print values
unsigned long printTime = 0;  // timer for printing stuff

#define LED_2 22
#define LED_1 23

#include <RadioLib.h>  // include radio library

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

bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;

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
bool wasTX = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
int lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int controls[] = {0, 0, 0, 0}; // stores arm flag, motor values, and latch state


void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);

  delay(1000);
}

void loop() {
  // error
  e = posi - target;

  // control signal
  u = kp*e;

  // motor power
  pwr = abs(u);
  if(pwr > 255){
    pwr = 255;
  }

  // motor direction
  dir = 1;
  if(u < 0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM_1,DIR_1);
}

void setMotor(int dir, int pwmVal, int pwmPin, int dirPin){
  analogWrite(pwmPin,pwmVal);
  
  if(dir > 0)
    digitalWrite(dirPin,HIGH);
  else
    digitalWrite(dirPin,LOW);
}

void readEncoder(){
  if(digitalRead(ENCB) > 0)
    posi++;
    
  else
    posi--;
}
