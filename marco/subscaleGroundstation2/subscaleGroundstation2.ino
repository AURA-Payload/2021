/*-------------------------------------------------------------
    Subscale ground code
    Written by Fowler Askew
    AURA Payload 2021-22
    Takes commands from serial port
    Transmits command array
    Receives telemetry string from rocket
    
    Commands:
    "ease" + "up" or "down" or "off"
    "soar" + "up" or "down" or "off"
    "sls" + "up" or "down" or "off"3
    "log" + "on" or "off"

    Outgoing data packet structure (to subscale):
    [SLS direction, SLS speed,
    SOAR direction, SOAR speed,
    EASE direction, EASE speed,
    datalogging toggle]

    Incoming data string structure (from subscale):
    "motor 1 speed, motor 2 speed, motor 3 speed,
    datalogging active, BMP altitude, RSSI at rocket"
    
    
    // TO DO
    // Add radio setup code
    // Add radio transmit code
    // Add serial command code
    // Add radio receive & print code
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
byte TXarray[] = {0, 0, 0, 0, 0, 0, 0};  // command array
bool stringComplete = false;  // flags when data is finished coming in
String inputString = "";  // holds incoming serial data

// Transmit/receive variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 1500;  // time between tranmissions
String receivedString = "";

// Radio variables
int transmitState = ERR_NONE;  // saves radio state when transmitting
int receiveState = ERR_NONE;  // saves radio state when receiving
volatile bool enableInterrupt = true;  // disables interrupt when not needed
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
                          15,  // output power (dBm)
                          8,  // preample length (symbols)
                          0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check

  delay(250);
  
  if (receiveState == ERR_NONE)  // if radio initialized correctly
    Serial.println(F("init success!"));
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
    while (true);  // stop the program
  }

  radio.setRfSwitchPins(RXENPIN, TXENPIN);  // set up RF switch pins
  radio.setDio0Action(setFlag);  // function that will be called when something is done

  transmitData();
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
      receiveState = radio.readData(receivedString);

      if (receiveState == ERR_NONE)  // packet received correctly
      {
        Serial.println(F("[SX1276] Received packet!"));
  
        // print data of the packet
        Serial.print(receivedString);
  
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

  if((millis() >= transmitTimer + transmitInterval) && txComplete)  // if the TX interval has passed and last TX is done
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
  transmitState = radio.startTransmit(TXarray, 7);  // transmit array

  Serial.print("Transmitting: ");
  Serial.print(TXarray[0]);
  Serial.print(", ");
  Serial.print(TXarray[1]);
  Serial.print(", ");
  Serial.print(TXarray[2]);
  Serial.print(", ");
  Serial.print(TXarray[3]);
  Serial.print(", ");
  Serial.print(TXarray[4]);
  Serial.print(", ");
  Serial.print(TXarray[5]);
  Serial.print(", ");
  Serial.println(TXarray[6]);

  transmitFlag = true;
  transmitTimer = millis();  // reset transmit timer
}

void handleCommand()
{
  enableInterrupt = false;
  int motorsLoc = inputString.indexOf("motors");
  int easeLoc = inputString.indexOf("ease");
  int soarLoc = inputString.indexOf("soar");
  int slsLoc = inputString.indexOf("sls");
  int logLoc = inputString.indexOf("log");
  int onLoc = inputString.indexOf("on");
  int offLoc = inputString.indexOf("off");
  int upLoc = inputString.indexOf("up");
  int downLoc = inputString.indexOf("down");
  bool txOn = true;

  if (motorsLoc > -1)
  {
    if (offLoc > -1)
    {
      TXarray[1] = 0;
      TXarray[3] = 0;
      TXarray[5] = 0;
    }
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (slsLoc > -1)  // SLS is TXarray[0:1]
  {
    if (offLoc > -1)
      TXarray[1] = 0;
    else if (upLoc > -1)
    {
      TXarray[0] = 0;
      TXarray[1] = 255;
    }
    else if (downLoc > -1)
    {
      TXarray[0] = 1;
      TXarray[1] = 255;
    }
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (soarLoc > -1)  // SOAR is TXarray[2:3]
  {
    if (offLoc > -1)
      TXarray[3] = 0;
    else if (upLoc > -1)
    {
      TXarray[2] = 0;
      TXarray[3] = 200;
    }
    else if (downLoc > -1)
    {
      TXarray[2] = 1;
      TXarray[3] = 200;
    }
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (easeLoc > -1)  // EASE is TXarray[4:5]
  {
    if (offLoc > -1)
      TXarray[5] = 0;
    else if (upLoc > -1)
    {
      TXarray[4] = 0;
      TXarray[5] = 255;
    }
    else if (downLoc > -1)
    {
      TXarray[4] = 1;
      TXarray[5] = 255;
    }
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (logLoc > -1)
  {
    if (onLoc > -1)
      TXarray[6] = 1;
    else if (offLoc > -1)
      TXarray[6] = 0;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else
  {
    txOn = false;
    Serial.println("Invalid Command");
  }

  if(txOn)
    transmitData();

  inputString = "";  // clear string
  stringComplete = false;  // reset flag
  enableInterrupt = true;
}
