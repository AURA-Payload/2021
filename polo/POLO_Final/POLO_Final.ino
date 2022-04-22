// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM_1 5
#define DIR_1 4

#define LED_2 22
#define LED_1 23

#define BIT0 8
#define BIT1 17
#define BIT2 20
#define BIT3 21

#define SENSE1 A9
#define SENSE2 A8

#define TONE_IN A2

#include <RadioLib.h>  // include radio library
#include <math.h>       
#include <TimeLib.h>

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

int sensorValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the value of all 32 sensors
int validTimes[32] = {700, 730, 800, 830, 900, 930, 1000, 1015, 1030, 1045, 1100, 1115, 1130, 1145, 1200, 1215, 1230, 1245, 1300, 1315, 1330, 1345, 1400, 1415, 1430, 1445, 1500, 1515, 1530, 1600, 1630, 1918};
int sensorVals[32] = {00000, 00001, 00010, 00011, 00100, 00101, 00110, 00111, 01000, 01001, 01010, 01011, 01100, 01101, 01110, 01111, 10000, 10001, 10010, 10011, 10100, 10101, 10110, 10111, 11000, 11001, 11010, 11011, 11100, 11101, 11110, 11111};
int sensorHour[] = {7,7,8,8,9,9,10,10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,16,16,17};  // stores the hour associated with each sensor
int sensorMinute[] = {0,30,0,30,0,30,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,0,30,0};  // stores the minutes associated with each sensor
int sunCalibration[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the maximum value for each sensor in the sun (preloaded with lowest possible ADC value)
int shadeCalibration[] = {1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,  // stores the minimum value for each sensor in the shade (peloaded with highest possible ADC value
                        1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,
                        1024,1024,1024,1024,1024,1024,1024,1024};
int midCalibration[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the middle value for each sensor


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
bool pointingNorth = false;

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

  pinMode(BIT0, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT3, OUTPUT);

  pinMode(SENSE1, INPUT);
  pinMode(SENSE2, INPUT);

  updateSelection(0);  // writes 0000 to sensor select pins

  delay(250);
  
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

  Serial.println("Place sundial in sunlight and press any key to calibrate");  // prompt user to calibrate for sunlight
  while(Serial.available() == 0)  // wait until something is typed
  
  calibrateSun();  // run the sunlight calibration
  Serial.println("Sunlight calibrated");  // indicate that sun calibration is done
  printArray(sunCalibration, 0, 31);  // print the calibration array
  
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(10);
  }

  delay(250);
  
  Serial.println("Place sundial in shade and press any key to calibrate");
  while(Serial.available() == 0)
  
  calibrateShade();  // run the shade calibration
  Serial.println("Shade calibrated");  // indicate that shade calibration is done
  printArray(shadeCalibration, 0, 31);  // print the calibration array

  
  
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(10);
  }

  for(int i = 0; i < 32; i++){
    midCalibration[i] = ((sunCalibration[i] + shadeCalibration[i])/2);
  }

  setSyncProvider(getTeensy3Time);  // I believe this syncs both RTCs on the Teensy

  delay(100);
  if (timeStatus()!= timeSet)
    Serial.println("Unable to sync with the RTC");
    
  else
    Serial.println("RTC has set the system time");

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
  if(directionFinding && !pointingNorth){
    int hours = hour();
    int minutes = minute();
    int timeVal = (hours * 100) + minutes;
    int sensor = findTime(timeVal);
    int found = 0;
    int sensorValue = -1;
    dir = 1;
    pwr = 0;
    if(sensor > -1){
      Serial.println(sensor);
    }
    if(sensor > -1 && found == 0) {
      sensorValue = readSensor(sensor);
      dir = 1;
      pwr = 30;
      Serial.println("Rotate...");
      if((midCalibration[sensor] -  2) <= sensorValue && sensorValue <= (midCalibration[sensor] + 2)){
        found = 1;
        pwr = 0;
        pointingNorth = true;
      }
    }
    setMotor(dir,pwr,PWM_1,DIR_1);
    if(found == 1){
      Serial.println("Found!");
      pointingNorth = true;
    }
  }
  
  if(pointingNorth && !directionFound){
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
    //Nothing right now, but if we need to do anything other than listen before finding range. 
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

void readSensors()  // reads all the sensors into the sensorValues array
{
  for(byte readLoop = 0; readLoop < 16; readLoop++)
  {
    updateSelection(readLoop);
    delayMicroseconds(200);  // this delay allows the analog signal to propage before being read (see POLO trello card for 'scope measurement)
    sensorValues[readLoop] = analogRead(SENSE1);  // read the values into an array
    sensorValues[readLoop + 16] = analogRead(SENSE2);
  }
}

int readSensor(byte senseIndex)  // returns the value of a specific sensor
{
  updateSelection(senseIndex);
  delayMicroseconds(200);  // this delay allows the analog signal to propage before being read (see POLO trello card for 'scope measurement)
  
  if(senseIndex & 0b00010000)  // if the index is in the 16-32 range
    return analogRead(SENSE2);
  else
    return analogRead(SENSE1);
}

void printArray(int arrayIn[], int printStart, int printEnd)  // print an array Use the array indexes you want (0-31 will print a 32 value array)
{
  for(byte printLoop = printStart; printLoop <= printEnd; printLoop++)
  {
    Serial.print(arrayIn[printLoop]);
    Serial.print("\t");
  }
  Serial.print('\n');
}

void updateSelection(byte selectVar) {
  bool bitVal = selectVar & 0b00000001; // set LSB
  digitalWrite(BIT0, bitVal);
  bitVal = selectVar & 0b00000010;  // set bit 1
  digitalWrite(BIT1, bitVal);
  bitVal = selectVar & 0b00000100;  // set bit 2
  digitalWrite(BIT2, bitVal);
  bitVal = selectVar & 0b00001000;  // set MSB
  digitalWrite(BIT3, bitVal);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

int findTime(int timeVal){
  for(int i = 0; i < 32; i++){
    if((timeVal + 100) == validTimes[i]){ //FOR DAYLIGHT SAVING TIME, I ADDED THE 100. Might need to be removed. 
      return i; 
    }
  }
  return -1; 
}
