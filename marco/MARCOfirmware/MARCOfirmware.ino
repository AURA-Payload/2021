/*-------------------------------------------------------------
    MARCO code
    AURA Payload 2021-22
    Takes commands from serial port
    Transmits command array
    Receives telemetry packets from payload
    
    Commands:
    "arm" + "on" or "off" -> this activates motor control
    "ease" + "up" or "down" or "off"
    "soar" + "up" or "down" or "off"
    "sls" + "up" or "down" or "off"
    "latch" + "on" or "off"
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
byte TXarray[] = {0, 0b00000000, 0, 0, 0, 0, 0, 0, 0};  // outgoing array
byte RXarray[] = {0, 0b00000000, 0, 0, 0, 0, 0, 0, 0};  // incoming array
volatile bool stringComplete = false;  // flags when user input is finished coming in
String inputString = "";  // holds serial data from PC
bool newCommand = false;

// Transmit/receive variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 1500;  // time between tranmissions

// Radio variables
int transmitState = ERR_NONE;  // saves radio state when transmitting
int receiveState = ERR_NONE;  // saves radio state when receiving
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
  
  if (receiveState == ERR_NONE)  // if radio initialized correctly
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
  if (stringComplete)
    handleCommand();
  
  if(operationDone)  // if the last operation is finished
  {
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      if (transmitState == ERR_NONE)  // if transmission completed successsfully
        Serial.println(F("transmission finished"));
      else  // if transmission failed
      {
        Serial.print(F("transmission failed, code "));
        Serial.println(transmitState);
      }

      transmitFlag = false;  // not transmitting this time
      txComplete = true;
    }

    else  // last action was receive
    {
      receiveState = radio.readData(RXarray, 9);  // save received data to RXarray

      if (receiveState == ERR_NONE)  // packet received correctly
      {
        Serial.println(F("[SX1276] Received packet!"));
  
        Serial.print("Received: ");
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
        Serial.print(", ");
        Serial.println(RXarray[8]);
  
        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("\t[SX1276] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
      }
      else if (receiveState == ERR_CRC_MISMATCH)  // packet received malformed
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

  if(txComplete && (newCommand || (millis() >= transmitTimer + transmitInterval)))  // if the TX interval has passed and last TX is done
    transmitData();
}

void serialEvent()
{
  while (Serial.available()) {
    char inChar = (char)Serial.read();  // get the new byte
    inputString += inChar;  // add to inputString
    if (inChar == '\n') {  // toggle flag when newline
      stringComplete = true;
    }
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
  transmitState = radio.startTransmit(TXarray, 9);  // transmit array

  Serial.print("Transmitting: ");
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
  Serial.print(TXarray[7]);
  Serial.print(", ");
  Serial.println(TXarray[8]);

  transmitFlag = true;
  transmitTimer = millis();  // reset transmit timer
}

void handleCommand()
{
  enableInterrupt = false;
  int armLoc = inputString.indexOf("arm");
  int easeLoc = inputString.indexOf("ease");
  int soarLoc = inputString.indexOf("soar");
  int slsLoc = inputString.indexOf("sls");
  int latchLoc = inputString.indexOf("latch");
  int onLoc = inputString.indexOf("on");
  int offLoc = inputString.indexOf("off");
  int upLoc = inputString.indexOf("up");
  int downLoc = inputString.indexOf("down");
  bool validCommand = true;

  if (armLoc > -1)
  {
    if (offLoc > -1)
    {
      TXarray[1] & 0b11111110;  // clear ARM bit
      TXarray[2] = 0;  // set EASE speed to 0
      TXarray[3] = 0;  // set SOAR speed to 0
      TXarray[4] = 0;  // set SLS speed to 0
    }
    else if (onLoc > -1)
      TXarray[1] | 0b00000001;  // set ARM bit
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (easeLoc > -1)
  {
    if (offLoc > -1)
      TXarray[2] = 0;
    else if (upLoc > -1)
    {
      TXarray[2] = 255;
      TXarray[1] | 0b00000010;
    }
    else if (downLoc > -1)
    {
      
      TXarray[2] = 255;
      TXarray[1] & 0b11111101;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (soarLoc > -1)
  {
    if (offLoc > -1)
      TXarray[3] = 0;
    else if (upLoc > -1)
    {
      TXarray[3] = 255;
      TXarray[1] | 0b00000100;
    }
    else if (downLoc > -1)
    {
      
      TXarray[3] = 255;
      TXarray[1] & 0b11111011;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (slsLoc > -1)
  {
    if (offLoc > -1)
      TXarray[4] = 0;
    else if (upLoc > -1)
    {
      TXarray[4] = 255;
      TXarray[1] | 0b00001000;
    }
    else if (downLoc > -1)
    {
      
      TXarray[4] = 255;
      TXarray[1] & 0b11110111;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }
  
  else if (latchLoc > -1)
  {
    if (offLoc > -1)
    {
      TXarray[1] & 0b11101111;  // clear latch bit
    }
    else if (onLoc > -1)
      TXarray[1] | 0b00010000;  // set latch bit
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else
  {
    validCommand = false;
    Serial.println("Invalid Command");
  }

  if(validCommand)  // If the command is valid
    newCommand = true;

  inputString = "";  // clear string
  stringComplete = false;  // reset flag
  enableInterrupt = true;
}
