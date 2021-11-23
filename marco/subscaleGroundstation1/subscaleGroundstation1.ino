/*-------------------------------------------------------------
    Subscale ground station code
    Written by Fowler Askew
    AURA Payload 2021-22
    Sends control commands to subscle board
    Receives data from subscale board and displays to team

    Commands:
    "motors" + "on" or "off"
    "nudge" + "on" or "off"
    "ease" + "up" or "down" or "off"
    "soar" + "up" or "down" or "off"
    "sls" + "up" or "down" or "off"
    "log" + "on" or "off"
    "tx" retransmits command array

    Outgoing data packet structure (to rocket):
    [motor enable, nudge mode,
    EASE command, SOAR command,
    SLS command, datalogging toggle]

    Incoming data packet structure (from rocket):
    [motor 1 info, motor 2 info,
    motor 3 info, datalogging active]

    Motor numbers: 0 - CW, 255 - CCW, else 0
  -------------------------------------------------------------*/
#include <RadioLib.h>

//  set up radio parameters
#define radioCarrier 915.0  // carrier frequency
#define radioBW 125.0  // bandwidth (kHz)
#define radioSF 9  // spreading factor
#define radioCR 7  // coding rate
#define radioSync 0x12  // sync word (same for TX and RX)
#define radioPower 15  // output power (dBm)
#define radioPreamble 8  // preamble length (symbols)
#define radioGain 0  // gain (0 is automatic control)
#define radioCRC true  // cyclic redundancy check

#define pin_cs 10  // radio chip select
#define pin_nrst 33  // radio reset
#define pin_dio0 34  // radio DIO0
#define pin_dio1 35  // radio DIO1
#define pin_rxen 36  // radio RX enable
#define pin_txen 37  // radio TX enable

SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);  // create radio object

byte commands[] = {0x0, 0x0, 0x7F, 0x7F, 0x7F, 0x0};  // create command array
unsigned int motor1Timer = 0;
unsigned int motor2Timer = 0;
unsigned int motor3Timer = 0;

bool stringComplete = false;  // flags when data is finished coming in
String inputString = "";  // holds incoming serial data

// RX interrupt setup
volatile bool receivedFlag = false;  // received flag
volatile bool enableInterrupt = true;  // is it allowed

void setFlag(void) {  // sets received flag
  if(!enableInterrupt)  // is it allowed
    return;

  receivedFlag = true;  // if there's a packet
}

void setup()
{
  inputString.reserve(200);  // make sure we have room
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Welcome to the Subscale Ground Station! Have a wonderful day!");

  Serial.print(F("[SX1276] Initializing ... "));  // initialize radio
  int state = radio.begin(radioCarrier, radioBW, radioSF, radioCR, radioSync, radioPower, radioPreamble, radioGain);

  if (state == ERR_NONE) {  // report if radio started correctly
    Serial.println(F("Radio boot success!"));
  } else {
    Serial.print(F("Radio boot failed, code "));
    Serial.println(state);
    Serial.println("Try rebooting");
    while (true);
  }

  radio.setCRC(radioCRC);  // set up CRC
  radio.setRfSwitchPins(pin_rxen, pin_txen);  // set up RF switch
  
  radio.setDio0Action(setFlag);  // set the function that will be called
  transmitArray();  // transmit the command array
}

// Loop
void loop()
{
  if(receivedFlag)
    printData();

  if (stringComplete)
    handleCommand();
}

void handleCommand()
{
  int motorsLoc = inputString.indexOf("motors");
  int nudgeLoc = inputString.indexOf("nudge");
  int easeLoc = inputString.indexOf("ease");
  int soarLoc = inputString.indexOf("soar");
  int slsLoc = inputString.indexOf("sls");
  int logLoc = inputString.indexOf("log");
  int onLoc = inputString.indexOf("on");
  int offLoc = inputString.indexOf("off");
  int upLoc = inputString.indexOf("up");
  int downLoc = inputString.indexOf("down");
  int txLoc = inputString.indexOf("tx");
  bool txOn = true;

  if (motorsLoc > -1)
  {
    if (offLoc > -1)
      commands[0] = 0x00;
    else if (onLoc > -1)
      commands[0] = 0xFF;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

//  else if (nudgeLoc > -1)
//  {
//    if (onLoc > -1)
//      commands[1] = 0xFF;
//    else if (offLoc > -1)
//      commands[1] = 0x00;
//    else
//    {
//      txOn = false;
//      Serial.println("Invalid Command");
//    }
//  }

  else if (slsLoc > -1)
  {
    if (offLoc > -1)
      commands[2] = 0x7F;
    else if (upLoc > -1)
      commands[2] = 0x00;
    else if (downLoc > -1)
      commands[2] = 0xFF;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (soarLoc > -1)
  {
    if (offLoc > -1)
      commands[3] = 0x7F;
    else if (upLoc > -1)
      commands[3] = 0x00;
    else if (downLoc > -1)
      commands[3] = 0xFF;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (easeLoc > -1)
  {
    if (offLoc > -1)
      commands[4] = 0x7F;
    else if (upLoc > -1)
      commands[4] = 0x00;
    else if (downLoc > -1)
      commands[4] = 0xFF;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (logLoc > -1)
  {
    if (onLoc > -1)
      commands[5] = 0xFF;
    else if (offLoc > -1)
      commands[5] = 0x00;
    else
    {
      txOn = false;
      Serial.println("Invalid Command");
    }
  }

  else if (txLoc > -1)
  {
    Serial.println("Transmitting");
  }

  else
  {
    txOn = false;
    Serial.println("Invalid Command");
  }

  if(txOn)
    transmitArray();

  inputString = "";  // clear string
  stringComplete = false;  // reset flag
}

void transmitArray()
{
  int state = radio.transmit(commands, 6);

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("Transmitted!"));

  Serial.print(commands[0]);
  Serial.print("\t");
  Serial.print(commands[1]);
  Serial.print("\t");
  Serial.print(commands[2]);
  Serial.print("\t");
  Serial.print(commands[3]);
  Serial.print("\t");
  Serial.print(commands[4]);
  Serial.print("\t");
  Serial.println(commands[5]);

    // print measured data rate
    /*Serial.print(F("[SX1276] Datarate:\t"));
      Serial.print(radio.getDataRate());
      Serial.println(F(" bps"));*/

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("Packet too long"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("TX timeout"));

  } else {
    // some other error occurred
    Serial.print(F("TX failed, code "));
    Serial.println(state);
  }

  beginListen();
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

void beginListen()
{
  volatile int state = radio.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("Now listening"));
  } else {
    Serial.print(F("RX failed, code "));
    Serial.println(state);
  }
}

void printData()  // gets the received data and outputs
{
  enableInterrupt = false;  // disable ISR
  receivedFlag = false;  // reset flag
  
  volatile byte telemetry[4];
  volatile int state = radio.readData(telemetry, 4);  // get data

  if (state == ERR_NONE) {  // if successful RX
    Serial.print("SLS state: ");
    Serial.println(telemetry[0]);
    Serial.print("SOAR state: ");
    Serial.println(telemetry[1]);
    Serial.print("EASE state: ");
    Serial.println(telemetry[2]);
    if(telemetry[3])
      Serial.println("Datalogging active");
    else
      Serial.println("Datalogging inactive");

    // print RSSI (Received Signal Strength Indicator)
    Serial.print(F("RSSI: "));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("RX CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("RX failed, code "));
    Serial.println(state);

  }

  enableInterrupt = true;  // enable ISR
  beginListen();
}
