/*-------------------------------------------------------------
    MARCO code
    AURA Payload 2021-22
    
    Commands:
    "motorArm" + "on" or "off" -> this activates motor control
    "payloadArm" + "on" or "off" -> this activates autonomous operation
    "calibrate" + "sun" or "shade" -> this calibrates the sundial
    "ease" + "up" or "down" or "off"
    "soar" + "up" or "down" or "off"
    "sls" + "up" or "down" or "off"
    "legs" + "up" or "down" or "off"

    Program Flow:
      Continually output radio control packets
      Continually output telemetry data to serial port
      Continually monitor serial for manual commands

      Update command matrix in response to manual command input
      Wait for packet to indicate payload deployment
      Begin sending rapid radio messages and ultrasonic pulses
      Wait until packet is received from POLO
      Begin sending simultaneous ultrasonic/radio messages
      Wait until packet is received from POLO with direction/bearing & grid box number
  -------------------------------------------------------------*/
  
#include <RadioLib.h>  // radio library

// radio connections on ground station board:
#define CSPIN 10
#define DIO0PIN 34
#define NRSTPIN 33
#define DIO1PIN 35
#define RXENPIN 36
#define TXENPIN 37

SX1276 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);

// Command stuff
byte TXarray[] = {0};  // outgoing array
byte RXarray[] = {0, 0b00000000, 0, 0, 0, 0, 0, 0};  // incoming array
volatile bool stringComplete = false;  // flags when user input is finished coming in
String inputString = "";  // holds serial data from PC
bool newCommand = false;
bool motorControl = false;

// Transmit/receive variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 100;  // time between tranmissions
unsigned int transmitBlankTime = 5;  // dead time after a transmission

// Radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done

void setup()
{
  inputString.reserve(200);  // make sure we have room
  Serial.begin(115200);
  delay(250); //Wait for board to be able to print

  Serial.print(F("[SX1276] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                          125.0,  // bandwidth (kHz)
                          9,  // spreading factor
                          7,  // coding rate
                          0x12,  // sync word
                          17,  // output power (dBm)
                          8,  // preamble length (symbols)
                          0);  // gain (0 is automatic control)
  radio.setRfSwitchPins(RXENPIN, TXENPIN);  // set up RF switch pins
  radio.setCRC(true);  // enable cyclic redundancy check

  delay(250);
  
  if (receiveState == RADIOLIB_ERR_NONE)  // if radio initialized correctly
    Serial.println(F("init success!"));
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
    while (true);  // stop the program
  }

  radio.setDio0Action(setFlag);  // function that will be called when something is done

  transmitData();  // send the first packet
}

void loop()
{
  
  if(operationDone)  // if the last operation is finished
  {
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      if (transmitState != RADIOLIB_ERR_NONE)  // if transmission failed
      {
        Serial.print(F("transmission failed, code "));
        Serial.println(transmitState);
      }
//      else  // if transmission good
//        Serial.println(F("transmission finished"));
      

      transmitFlag = false;  // not transmitting this time
      txComplete = true;
    }

    else  // last action was receive
    {
      receiveState = radio.readData(RXarray, 8);  // save received data to RXarray

      if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
      {
//        Serial.println(F("[SX1276] Received packet!"));
//  
        Serial.print("   Received: ");
        Serial.print(RXarray[0]);
        Serial.print(", ");
        Serial.print(RXarray[1], BIN);
        Serial.print(", ");
        Serial.print(RXarray[2]);
        Serial.print(", ");
        Serial.print(RXarray[3]);
        Serial.print(", ");
        Serial.print(RXarray[4]);
        Serial.print(", ");
        Serial.print(RXarray[5]);
        Serial.print(", ");
        Serial.print(RXarray[6]);
        Serial.print(", ");
        Serial.print(RXarray[7]);
  
        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("\t[SX1276] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
      }
      else if (receiveState == RADIOLIB_ERR_CRC_MISMATCH)  // packet received malformed
        Serial.println(F("[SX1276] CRC error!"));
      else  // some other error
      {
        Serial.print(F("[SX1276] RX failed, code "));
        Serial.println(receiveState);
      }
    }

    enableInterrupt = true;  // reenable the interrupt
    receiveState = radio.startReceive();  // start receiving again
  }

  if(txComplete && ((newCommand && millis() >= transmitTimer + transmitBlankTime) || (millis() >= transmitTimer + transmitInterval)))  // if the TX interval has passed and last TX is done
  {
    newCommand = false;
    transmitData();
  }
}

void setFlag(void)  // this function is called after complete packet transmission or reception
{
  if(!enableInterrupt)  // check if the interrupt is enabled
    return;

  operationDone = true;  // something happened, set the flag
}

void transmitData()
{
  txComplete = false;
  transmitState = radio.startTransmit(TXarray, 8);  // transmit array

  Serial.print("Transmitted: ");
  Serial.print(TXarray[0]);
  Serial.print(", ");
  Serial.print(TXarray[1], BIN);
  Serial.print(", ");
  Serial.print(TXarray[2]);
  Serial.print(", ");
  Serial.print(TXarray[3]);
  Serial.print(", ");
  Serial.print(TXarray[4]);
  Serial.print(", ");
  Serial.print(TXarray[5]);
  Serial.print(", ");
  Serial.print(TXarray[6]);
  Serial.print(", ");
  Serial.println(TXarray[7]);

  transmitFlag = true;
  transmitTimer = millis();  // reset transmit timer
}
