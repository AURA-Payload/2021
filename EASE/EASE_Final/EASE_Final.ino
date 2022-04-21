// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM_1 9  // motorA is 9, 8; motorB is 6, 5
#define DIR_1 8
#define LED_1 18
#define LED_2 19

#include <RadioLib.h>  // include radio library

// RFM95 connections:
#define CSPIN 10
#define DIO0PIN 3
#define NRSTPIN 2
#define DIO1PIN 4

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

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

// control variables
int controls[] = {0, 0, 0}; // stores arm flag and motor values

bool deployed = false;

volatile int posi = 0; // specify posi as volatile

int kp = 5;
int pos = 0;
int e = 0;
int u = 0;
int pwr = 0;
int dir = 1;

int totalDistance = 17; //number of motor shaft rotations to complete
int gearRatio = 349; // for testing output shaft accuracy: 302 for POLO, 349 for EASE
//int gearRatio = 1;  // for testing small distance accuracy
int pulsePerRotate = 12;  // encoder pulses (rising) for one rotation: 7 for POLO, 11 for EASE
int screwPitch = 8;  // TPI of leadscrew
int totalRotations = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // sets the target to hit (should be 1 shaft rotation)
int target = 0;

int targetInterval = 3000;  // 2.5 seconds between switching targets
unsigned long targetSwitch = 0;
int printerval = 10;  // millisecond interval to print values
unsigned long printTime = 0;  // timer for printing stuff

void setup() {
  Serial.begin(115200);
  //pinMode(ENCA,INPUT);
  //pinMode(ENCB,INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  // ----- BEGIN RADIO SETUP -----
  // initialize RFM95 with all settings listed
  Serial.print(F("[RFM95] Initializing ... "));
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

  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  // ----- END RADIO SETUP -----
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

  if(operationDone)  // if the last operation is finished
  {
    digitalWrite(LED_1, LOW);  // No LEDs in between modes
    digitalWrite(LED_2, LOW);  // No LEDs in between modes
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
      setMotor(dir,pwr,PWM_1,DIR_1); // sets the motors based on controls array
      delay(10);
      //transmitData();  // send a message back to GS
    }
    receiveState = radio.startReceive();  // start receiving again
    enableInterrupt = true;  // reenable the interrupt
  }
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
    Serial.println(F("[RFM95] Received packet!"));

    Serial.print(F("[RFM95] Data:\t\t"));  // print data
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
    
    Serial.print(F("\t[RFM95] RSSI: "));  // print RSSI if desired
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
   
    //controls[0] = RXarray[1] & 0b00000001;  // controls[0] is set to the state of the arm bit

    pwr = abs(RXarray[2]);  // motor speed from RXarray
    if(~RXarray[1] & 0b00000010)  // if direction bit is not set
      dir *= -1;
    else{
      dir = 1;
    }
    
    if(RXarray[0] == 2 && RXarray[2] == 255){
      //target = totalRotations; 
      deployed = true;
      Serial.println("deployed");
      transmitData();
    }
      
  }

  setMotor(dir,pwr,PWM_1,DIR_1);
  
//  else if (receiveState == RADIOLIB_ERR_CRC_MISMATCH)  // packet received malformed
//    Serial.println(F("[RFM95] CRC error!"));
//  else  // some other error
//  {
//    Serial.print(F("[RFM95] Failed, code "));
//    Serial.println(receiveState);
//  }

}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 1;  // set EASE's system address
  
  transmitFlag = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  if(deployed){
    RXarray[2] = 1;
  }
  
  Serial.println(F("[RFM95] Sending array ... "));
  transmitState = radio.startTransmit(RXarray, 8);  // transmit array
  digitalWrite(LED_2, HIGH);  // LED 2 on while transmit mode is active
}
