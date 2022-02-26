/*-------------------------------------------------------------
    EASE Board Testing
    Devin Spivey
    AURA Payload 2021-22
  "
  -------------------------------------------------------------*/
#define LED_1 18
#define LED_2 19
#define LIMIT_1 16
#define LIMIT_2 17

#include <RadioLib.h>  // include radio library

// motor pins
#define PWM_B 6 // PWM pin, motor A
#define DIR_B 5  // Direction pin, motor A
#define PWM_A 9  // PWM pin, motor B
#define DIR_A 8  // Direction pin, motor B

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
byte RXarray[9];  // stores received array

// radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done

// control variables
int controls[] = {0, 0, 0}; // stores arm flag and motor values

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  pinMode(LIMIT_1, INPUT_PULLUP);
  pinMode(LIMIT_2, INPUT_PULLUP);

  Serial.begin(115200);
  delay(250);

  digitalWrite(DIR_B, LOW);  // disable motor B, we don't need it
  digitalWrite(PWM_B, LOW);

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

  setMotor();  // sets the motors based on controls array
}

void loop()
{

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
      setMotor();  // sets the motors based on controls array
      delay(10);
      //transmitData();  // send a message back to GS
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
//    Serial.println(F("[RFM95] Received packet!"));
//
//    Serial.print(F("[RFM95] Data:\t\t"));  // print data
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
//    Serial.print(F("\t[RFM95] RSSI: "));  // print RSSI if desired
//    Serial.print(radio.getRSSI());
//    Serial.println(F(" dBm"));
   
    controls[0] = RXarray[1] & 0b00000001;  // controls[0] is set to the state of the arm bit

    controls[1] = RXarray[2];  // motor speed from RXarray
    if(~RXarray[1] & 0b00000010)  // if direction bit is not set
      controls[1] *= -1;
      
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
  transmitTimer = millis();  // reset transmit timer
  
//  Serial.println(F("[RFM95] Sending array ... "));
  transmitState = radio.startTransmit(RXarray, 9);  // transmit array
  digitalWrite(LED_2, HIGH);  // LED 2 on while transmit mode is active
}

void setMotor()
{
  if(controls[0])
  {
    if(controls[1] > 0)  // speed is positive
      digitalWrite(DIR_A, HIGH);
    else
      digitalWrite(DIR_A, LOW);
      
    analogWrite(PWM_A, abs(controls[1]));
  }
  else
    analogWrite(PWM_A, 0);
}
