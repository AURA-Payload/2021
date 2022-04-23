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

#define TONE_PIN 5  // tone is generated on pin 5
#define FREQ 25000  // 25kHz

SX1276 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);

// Command stuff
byte TXarray[] = {0, 0b00000000, 0, 0, 0, 0, 0, 0};  // outgoing array
byte RXarray[] = {0, 0b00000000, 0, 0, 0, 0, 0, 0};  // incoming array
int packetLength = 8;  // define the length of the packet
volatile bool stringComplete = false;  // flags when user input is finished coming in
String inputString = "";  // holds serial data from PC
bool newCommand = false;
bool motorControl = false;  // is motor control active?
byte rangeState = 0;  // 1 = direction find started, 2 = ranging started, 3 = ranging complete

// Transmit/receive variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 50;  // time between tranmissions
unsigned int transmitBlankTime = 5;  // dead time after a transmission

unsigned int beaconTimer = 0;
unsigned int beaconDuration = 25;  // how long to transmit ultrasound for beacon (ms)
unsigned int directionInterval = 250;  // how often to transmit beacon signals for dir finding (ms)
unsigned int rangeInterval = 4000;  // how often to transmit beacon signals for ranging (ms)
bool isEmitting = false;  // indicates when the system is transmitting ultrasound
byte beaconPacket[] = {255};  // single byte packet for ranging operations

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
                          2,  // output power (dBm)
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
  if (stringComplete)  // if there's an input from PC
    handleCommand();
  
  if(operationDone)  // if the last radio operation is finished
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
      

      transmitFlag = false;  // transmitting is finished
      txComplete = true;
    }

    else  // last action was receive
    {
      receiveState = radio.readData(RXarray, packetLength);  // save received data to RXarray

      if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
        handleReceive();
        
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

  //if(rangeState == 0 && txComplete && ((newCommand && millis() >= transmitTimer + transmitBlankTime) || (millis() >= transmitTimer + transmitInterval)))
  if(rangeState == 0 && txComplete && (millis() >= transmitTimer + transmitInterval))
  {
    // if we have not entered the ranging mode
    // if the radio is done with the last transmission
    // if there is a new serial command and blanking time has passed or the TX interval has passed
    newCommand = false;
    transmitData();
  }

  else if(rangeState == 1)
  {
    // if the direction finding mode is active
    if(millis()-beaconTimer > directionInterval)  // if the interval has passed
    {
      txComplete = false;
      transmitFlag = true;
      transmitState = radio.startTransmit(beaconPacket, 1);  // transmit one byte
      tone(TONE_PIN, FREQ);  // start emitting the tone
      isEmitting = true;  // we are emitting sound
      beaconTimer = millis();  // reset transmit timer
    }
    
    if(isEmitting && millis()-beaconTimer > beaconDuration)
    {  // if there is ultrasound transmitting and we pass the output duration
      noTone(TONE_PIN);  // stop transmitting ultrasound
      isEmitting = false;
    }
  }

  else if(rangeState == 2)
  {
    // if the rangefinding mode is active
    if(millis()-beaconTimer > rangeInterval)  // if the interval has passed
    {
      txComplete = false;
      transmitFlag = true;
      transmitState = radio.startTransmit(beaconPacket, 1);  // transmit one byte
      tone(TONE_PIN, FREQ);  // start emitting the tone
      isEmitting = true;  // we are emitting sound
      beaconTimer = millis();  // reset transmit timer
    }
    
    if(isEmitting && millis()-beaconTimer > beaconDuration)
    {  // if there is ultrasound transmitting and we pass the output duration
      noTone(TONE_PIN);  // stop transmitting ultrasound
      isEmitting = false;
    }
  }
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

void transmitData()  // this transmit sends the packet
{
  txComplete = false;
  transmitState = radio.startTransmit(TXarray, packetLength);  // transmit array

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

void handleReceive()
{
  if(rangeState == 0)  // if ranging has not started, just print the received stuff & check for ranging begin
  {
    Serial.println(F("[SX1276] Received packet!"));
    
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
    
    if(RXarray[0] == 2 && (RXarray[1] & 0b10000000))  // if the transmission is from SOAR and range/direction find bit is set
    {
      rangeState = 1;  // set rangeState to the first stage (direction finding)
      TXarray[1] |= 0b10000000;  // set the direction finding bit as an acknowledgement
      Serial.println("Direction finding mode activated");
    }
  }

  else if(rangeState == 1)
  {
    // This is during the direction finding part
  }

  else if(rangeState == 2)
  {
    // This is during/after the ranging
  }
}

void handleCommand()  //ADD A COMMAND FOR MANUALLY ENDING/BEGINNING RANGING AND DIRECTION FINDING
{
  enableInterrupt = false;
  int motorLoc = inputString.indexOf("motorArm");
  int armLoc = inputString.indexOf("payloadArm");
  int calLoc = inputString.indexOf("calibrate");
  int easeLoc = inputString.indexOf("ease");
  int soarLoc = inputString.indexOf("soar");
  int slsLoc = inputString.indexOf("sls");
  int legsLoc = inputString.indexOf("legs");
  int onLoc = inputString.indexOf("on");
  int offLoc = inputString.indexOf("off");
  int upLoc = inputString.indexOf("up");
  int downLoc = inputString.indexOf("down");
  int sunLoc = inputString.indexOf("sun");
  int shadeLoc = inputString.indexOf("shade");
  bool validCommand = true;

  if (motorLoc > -1)
  {
    if (offLoc > -1)
    {
      motorControl = false;
      TXarray[2] = 0;  // set EASE speed to 0
      TXarray[3] = 0;  // set SOAR speed to 0
      TXarray[4] = 0;  // set SLS speed to 0
      TXarray[5] = 0;  // set LEGS1 speed to 0
      TXarray[6] = 0;  // set LEGS2 speed to 0
    }
    else if (onLoc > -1)
    {
      motorControl = true;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (armLoc > -1)
  {
    if (offLoc > -1)
    {
      TXarray[1] &= 0b11111110;  // clear ARM bit
      TXarray[2] = 0;  // set EASE speed to 0
      TXarray[3] = 0;  // set SOAR speed to 0
      TXarray[4] = 0;  // set SLS speed to 0
      TXarray[5] = 0;  // set LEGS1 speed to 0
      TXarray[6] = 0;  // set LEGS2 speed to 0
    }
    else if (onLoc > -1)
    {
      TXarray[1] |= 0b00000001;  // set ARM bit
      TXarray[2] = 0;  // set EASE speed to 0
      TXarray[3] = 0;  // set SOAR speed to 0
      TXarray[4] = 0;  // set SLS speed to 0
      TXarray[5] = 0;  // set LEGS1 speed to 0
      TXarray[6] = 0;  // set LEGS2 speed to 0
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (easeLoc > -1)
  {
    if (!motorControl || offLoc > -1)
      TXarray[2] = 0;
    else if (upLoc > -1)
    {
      TXarray[2] = 255;
      TXarray[1] |= 0b00000010;
    }
    else if (downLoc > -1)
    {
      
      TXarray[2] = 255;
      TXarray[1] &= 0b11111101;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (soarLoc > -1)
  {
    if (!motorControl || offLoc > -1)
      TXarray[3] = 0;
    else if (upLoc > -1)
    {
      TXarray[3] = 255;
      TXarray[1] |= 0b00000100;
    }
    else if (downLoc > -1)
    {
      
      TXarray[3] = 255;
      TXarray[1] &= 0b11111011;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (slsLoc > -1)
  {
    if (!motorControl || offLoc > -1)
      TXarray[4] = 0;
    else if (upLoc > -1)
    {
      TXarray[4] = 255;
      TXarray[1] |= 0b00001000;
    }
    else if (downLoc > -1)
    {
      TXarray[4] = 100;
      TXarray[1] &= 0b11110111;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else if (legsLoc > -1)
  {
    if (!motorControl || offLoc > -1)
    {
      TXarray[5] = 0;
      TXarray[6] = 0;
    }
    else if (upLoc > -1)
    {
      TXarray[5] = 30;
      TXarray[6] = 30;
      TXarray[1] |= 0b00010000;
    }
    else if (downLoc > -1)
    {
      TXarray[5] = 30;
      TXarray[6] = 30;
      TXarray[1] &= 0b11101111;
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }
  
  else if (calLoc > -1)
  {
    if (offLoc > -1)
    {
      TXarray[1] &= 0b10011111;  // clear calibrate bits
    }
    else if (sunLoc > -1)
    {
      TXarray[1] |= 0b00100000;  // set cal bit
      TXarray[1] &= 0b10111111;  // clear sun/shade bit
    }
    else if (shadeLoc > -1)
    {
      TXarray[1] |= 0b01100000;  // set cal bit & sun/shade bit
    }
    else
    {
      validCommand = false;
      Serial.println("Invalid Command");
    }
  }

  else
  {
    TXarray[1] &= 0b10011111;  // clear cal & sun/shade bit
    validCommand = false;
    Serial.println("Invalid Command");
  }

  if(validCommand)  // If the command is valid
    newCommand = true;

  inputString = "";  // clear string
  stringComplete = false;  // reset flag
  enableInterrupt = true;
}
