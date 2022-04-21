// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM_1 5
#define DIR_1 4

#define LED_2 22
#define LED_1 23

#define TONE_IN A2

#include <RadioLib.h>  // include radio library
#include <math.h>       

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

#define transmitDelay 10  // how many milliseconds to wait before transmitting stuff


unsigned long transmitTimer = 0;  // stores the time of the last transmission
unsigned long transmitInterval = 2000;  // stores the time of the last transmission
unsigned int receiveTime = -1;
unsigned int analogTime = -1; 
float distance = 0; 
int rounds = 0;

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

float currentRSSI = -1000000;
float bestRSSI = -1000000;
int bestPosi = -1;
int marcoDirection = -1; 

bool directionFound = false;
bool directionFinding = false;
bool directed = false;
bool radioReceived = false;
bool calculated = false;

String gridString;

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;

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
  target = totalRotations;
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(PWM_A, LOW);
  digitalWrite(DIR_A, LOW);
  digitalWrite(PWM_B, LOW);
  digitalWrite(DIR_B, LOW);

  Serial.begin(115200);
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
  
  Serial.println("Startup complete");
  delay(250);
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  digitalWrite(LED_2, LOW);

  delay(1000);
}

void loop() {
  if(directionFinding && !directionFound){
    if(totalRotations <= posi){
      marcoDirection = bestPosi;
      directionFound = true; 
    }
    currentRSSI = radio.getRSSI();
    if(currentRSSI > bestRSSI){
      bestRSSI = currentRSSI;
      bestPosi = posi;
      Serial.print("Best RSSI: ");
      Serial.println(bestRSSI); 
      Serial.print("Best Posi: ");
      Serial.println(bestPosi);
    }
    dir = 1;
    pwr = 30;

    // signal the motor
    setMotor(dir,pwr,PWM_1,DIR_1);
  }
  
  if(directionFound && !directed){
    target = marcoDirection;
    
    // error
    e = posi - target;
    
    if(abs(e) < 5){
      directed = true; 
    }
    
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

  //Range finding
  if(directed){
    
  }

  if(radioReceived && analogRead(TONE_IN) >= 300){
    float roundDistance;
    rounds++;
    analogTime = micros();
    radioReceived = false;
    roundDistance = ((analogTime - receiveTime) * (.001125));
    distance = (roundDistance + distance)/rounds;
    if(rounds >= 3){
      calculateGrid();
    }
  }
  
  if(operationDone)  // if the last operation is finished
  {
    digitalWrite(LED_1, LOW);  // No LED in between modes
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(wasTX)  // last action was transmit
    {
      wasTX = false;  // not transmitting this time
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

  if(directionFound){
    if(/*(!hasTransmitted && millis() - receiveTime >= transmitDelay) || */millis() - transmitTimer >= transmitInterval)
    {
      transmitData();
    }
  }

  if(calculated){
    transmitGrid();
  }
}

void transmitGrid(){
  RXarray[0] = 3;  // set POLO's system address
  
  wasTX = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
  //  Serial.println(F("[RFM97] Sending array ... "));
  transmitState = radio.startTransmit(gridString);  // transmit array
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

void calculateGrid(){
  float theta = marcoDirection * (0.1703);
  float xDist;
  float yDist;

  float groundX = 0;
  float groundY = 0;

  float finalX;
  float finalY;

  int gridX;
  int gridY;
  int gridBox;
  
  if(theta == 0 || theta == 360){
    xDist = 0;
    yDist = -(distance);
  }
  else if(theta < 90){
    xDist = (distance * sin(theta));
    yDist = -(distance * cos(theta));
  }
  else if(theta == 90){
    xDist = (distance);
    yDist = 0;
  }
  else if(theta < 180){
    theta = theta - 90;
    xDist = (distance * cos(theta));
    yDist = (distance * sin(theta));
  }
  else if(theta == 180){
    xDist = 0;
    yDist = (distance);
  }
  else if(theta < 270){
    theta = theta - 180;
    xDist = -(distance * cos(theta));
    yDist = (distance * sin(theta));
  }
  else if(theta == 270){
    xDist = -(distance);
    yDist = 0;
  }
  else {
    theta = theta - 270;
    xDist = -(distance * sin(theta));
    yDist = -(distance * cos(theta));
  }

  finalX = xDist + groundX;
  finalY = yDist + groundY;

  gridX = finalX/250;
  gridY = finalY/250;

  gridBox = 1 + gridX + (gridY * 20);

  gridString = String(gridBox);
  calculated = true;
}

void setFlag(void)  // this function is called after complete packet transmission or reception
{
  if(!enableInterrupt)  // check if the interrupt is enabled
    return;
  if(directed){
    receiveTime = micros();
    radioReceived = true; 
  }
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

    if(RXarray[0] = 2){ //Only do anything if it comes from SOAR
      if(RXarray[1] & 0b00100000){
        if(RXarray[1] & 0b01000000){
          calibrateSun();
        }
        else{
          calibrateShade();
        }
      }
      if(RXarray[1] & 0b100000000){
        directionFinding = true; 
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
  
  wasTX = true;
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
