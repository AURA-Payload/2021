/*-------------------------------------------------------------
  POLO code for radio comms test
  Devin Spivey and Fowler Askew
  AURA Payload 2021-22
  -------------------------------------------------------------*/
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

void setup()
{
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
}

void loop()
{
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

    if(RXarray[0] = 2){ //Only do anything if it comes from SOAR
      if(RXarray[1] & 0b00100000){
        if(RXarray[1] & 0b01000000){
          calibrateSun();
        }
        else{
          calibrateShade();
        }
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
