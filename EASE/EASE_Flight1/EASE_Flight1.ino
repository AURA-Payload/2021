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

  Serial.begin(115200);
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

  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  // ----- END RADIO SETUP -----

  setMotors();  // sets the motors based on controls array
}

void loop()
{

  if(operationDone)  // if the last operation is finished
  {
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      transmitFlag = false;  // not transmitting this time
      txComplete = true;
      receiveState = radio.startReceive();  // start receiving again
    }

    else  // last action was receive
    {
      handleReceive();  // this stores received data to RXarray and saves RSSI
      setMotors();  // sets the motors based on controls array
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
  receiveState = radio.readData(RXarray, 7);  // read received data to array

  if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
  {
//    Serial.println(F("[RFM95] Received packet!"));
//
//    Serial.print(F("[RFM95] Data:\t\t"));  // print data
//    Serial.print(RXarray[0]);
//    Serial.print("\t");
//    Serial.print(RXarray[1]);
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
    
    lastRSSI = radio.getRSSI();  // get RSSI to be reported
    
//    Serial.print(F("\t[RFM95] RSSI: "));  // print RSSI if desired
//    Serial.print(lastRSSI);
//    Serial.println(F(" dBm"));
   
    if(RXarray[0])
      controls[0] = -1 * RXarray[1];
    else
      controls[0] = RXarray[1];
    
    if(RXarray[2])
      controls[1] = -1 * RXarray[3];
    else
      controls[1] = RXarray[3];
    
    if(RXarray[4])
      controls[2] = -1 * RXarray[5];
    else
      controls[2] = RXarray[5];

    controls[3] = RXarray[6];
  }
//  else if (receiveState == ERR_CRC_MISMATCH)  // packet received malformed
//    Serial.println(F("[RFM95] CRC error!"));
//  else  // some other error
//  {
//    Serial.print(F("[RFM95] Failed, code "));
//    Serial.println(receiveState);
//  }
}

void transmitData()
{
  String telemetryString = "";
  
  telemetryString += String(controls[0]);
  telemetryString += ", ";
  telemetryString += String(controls[1]);
  telemetryString += ", ";
  telemetryString += String(controls[2]);
  telemetryString += ", ";
  telemetryString += String(controls[3]);
  telemetryString += ", ";
  telemetryString += String(lastRSSI);
  telemetryString += "dBm\n";
  
  transmitFlag = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
  //Serial.println(F("[RFM95] Sending string ... "));
  transmitState = radio.startTransmit(telemetryString);  // transmit array 1
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