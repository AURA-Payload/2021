// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#include <RadioLib.h>  // include radio library

#define ENCA 17 // interrupt on pin 17 (PC3)
#define ENCB 16 // non-interrupted sensor
#define PWM_1 9  // motorA is 9, 8; motorB is 6, 5
#define DIR_1 8
#define LED_1 18
#define LED_2 19

// RFM95 connections:
#define CSPIN 10
#define DIO0PIN 3
#define NRSTPIN 2
#define DIO1PIN 4

#define totalDistance 17 //number of motor shaft rotations to complete
#define gearRatio 326 // for testing output shaft accuracy: 302 for POLO, 349 for EASE
//#define gearRatio 1  // for testing small distance accuracy
#define pulsePerRotate 11  // encoder pulses (rising) for one rotation: 7 for POLO, 11 for EASE
#define screwPitch 8  // TPI of leadscrew

#define transmitDelay 10  // how many milliseconds to wait before transmitting stuff
#define transmitInterval 1000  // milliseconds between transmissions

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object

// transmit variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int receiveTime = 0;  // stores the time when a packet was received
bool hasTransmitted = true;  // flag to keep track of when transmissions need to be made

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
int motorSpeed = 0;
bool isArmed = false;  // stores arm state (is auto deployment allowed)
bool isDeploying = false;  // indcates if we're currently ejecting
bool deployed = false;   // indicates when deployment is finished

volatile unsigned int posi = 0; // specify posi as volatile
unsigned int target = totalDistance * gearRatio * pulsePerRotate * screwPitch;
// sets the target to hit (should be 1 shaft rotation)

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  setMotor(motorSpeed);  // turn the motor off
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  //PCICR |= 0b00000010;  // enable interrupts on PC register
  //PCMSK1 |= 0b00001000;  // use interrupt mask on D17/A3/PC3

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

void loop(){
  if(isDeploying){  // if we're currently deploying
    if(posi < target)  // we haven't reached the target distance
      motorSpeed = 255;  // run the motor
    else  // if we have reached the target
    {
      motorSpeed = 0;  // stop the motor
      isDeploying = false;  // stop deploying
      deployed = true;
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
      //receiveState = radio.startReceive();  // start receiving again
      digitalWrite(LED_1, HIGH);  // LED 1 on while receive mode is active
    }

    else{  // last action was receive
      handleReceive();  // this stores received data to RXarray and saves RSSI
    }
    Serial.println("Listening for packets");
    receiveState = radio.startReceive();  // start receiving again
    enableInterrupt = true;  // reenable the interrupt
  }

  if((!hasTransmitted && millis() - receiveTime >= transmitDelay) || millis() - transmitTimer >= transmitInterval)
    transmitData();
  
  setMotor(motorSpeed);  // update the motor
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


    receiveTime = millis();  // store the time when data was received
    hasTransmitted = false;  // indicate that we have not retransmitted stuff

    if(RXarray[0] == 0){  // if command is from MARCO
      isArmed = RXarray[1] & 0b00000001;  // set isArmed to arming bit (can disable during a deployment)
      
      if(!isDeploying){  // if it's not in the middle of an autonomous deployment
        motorSpeed = RXarray[2];  // motor speed from RXarray
        if(~RXarray[1] & 0b00000010)  // if direction bit is not set
          motorSpeed *= -1;
      }
      else if(!isArmed)  // is it is currently deploying and has been disarmed
      {
        isDeploying = false;  // stop deploying
        motorSpeed = 0;  // set motorSpeed to 0
      }
    }
    
    if(RXarray[0] == 2){  // if command is from SOAR
      if(RXarray[2] == 255)  // if the EASE byte is set
      {
        deployed = true;
        Serial.println("deployed");
      }
    }
  }
  
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
  transmitTimer = millis();
  hasTransmitted = true;
  
  if(deployed){
    RXarray[2] = 1;
  }
  
  Serial.print(F("[RFM95] Sending array ... "));
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

void setMotor(int pwmVal)  // just sets the motor based on the PWM value
{
  analogWrite(PWM_1,abs(pwmVal));
  
  if(pwmVal > 0)
    digitalWrite(DIR_1,HIGH);
  else
    digitalWrite(DIR_1,LOW);
}

ISR (PCINT1_vect){  // encoder read function
  if(PINC & 0b00001000)  // if interrupt is high (rising signal)
  {
    if(PINC & 0b00000100)  // if PC2 (DIO16) is high
      posi++;  // increment position
    
    else  // is PC2 is low
      posi--;  // decrement position
  }
}
