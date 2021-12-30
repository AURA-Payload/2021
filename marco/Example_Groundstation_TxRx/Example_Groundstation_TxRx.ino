/*
  Modifies Radiolib's SX127x_PingPong

  Transmits 8 byte arrays periodically
  Listens for 8 byte arrays from subscale board
  
  Default parameters are:
  Carrier frequency: 434.0 MHz
  Bandwidth: 125.0 kHz
  Spreading factor: 9
  Coding rate: 4/7
  Sync word: 0x12
  Output power: 10 dBm
  Preamble length: 8 symbols
  Gain: 0(automatic gain control)
*/

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib

// These are the connections used on my ground station board:
#define CSPIN 10
#define DIO0PIN 34
#define NRSTPIN 33
#define DIO1PIN 35
#define RXENPIN 36
#define TXENPIN 37

SX1276 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);

// Transmit variable
byte TXarray1[] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F};
byte TXarray2[] = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10};
byte TXarraySelect = 0;  // 0 is TXarray1, else TXarray2
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 1000;  // time between tranmissions

// Receive array
byte RXarray[8];  // stores received array

// Radio variables
int transmitState = ERR_NONE;  // saves radio state when transmitting
int receiveState = ERR_NONE;  // saves radio state when receiving
volatile bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool transmitFlag = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done

void setup()
{
  Serial.begin(115200);
  delay(500); //Wait for board to be able to print

  Serial.print(F("[SX1276] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                          125.0,  // bandwidth (kHz)
                          9,  // spreading factor
                          7,  // coding rate
                          0x12,  // sync word
                          17,  // output power (dBm)
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
  if(operationDone)  // if the last operation is finished
  {
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(transmitFlag)  // last action was transmit
    {
      if (transmitState == ERR_NONE)  // if transmission completed successsfully
        Serial.println(F("transmission finished!"));
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
      receiveState = radio.readData(RXarray, 8);

      if (receiveState == ERR_NONE)  // packet received correctly
      {
        Serial.println(F("[SX1276] Received packet!"));
  
        // print data of the packet
        Serial.print(F("[SX1276] Data:\t\t"));
        Serial.print(RXarray[0]);
        Serial.print("\t");
        Serial.print(RXarray[1]);
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
  
        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("\t[SX1276] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
  //
  //      // print SNR (Signal-to-Noise Ratio)
  //      Serial.print(F("[SX1276] SNR:\t\t"));
  //      Serial.print(radio.getSNR());
  //      Serial.println(F(" dB"));
  //
  //      // print frequency error
  //      Serial.print(F("[SX1276] Frequency error:\t"));
  //      Serial.print(radio.getFrequencyError());
  //      Serial.println(F(" Hz"));
      }
      else if (receiveState == ERR_CRC_MISMATCH)  // packet received malformed
        Serial.println(F("[SX1276] CRC error!"));
      else  // some other error
      {
        Serial.print(F("[SX1276] Failed, code "));
        Serial.println(receiveState);
      }
    }

    enableInterrupt = true;  // reenable the interrupt
    receiveState = radio.startReceive();  // start receiving again
  }

  if((millis() >= transmitTimer + transmitInterval) && txComplete)  // if the TX interval has passed and last TX is done
    transmitData();
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
  Serial.println(F("[SX1276] Sending packet ... "));
  if(TXarraySelect)  // choose array to transmit based on TXarraySelect
    transmitState = radio.startTransmit(TXarray2, 8);  // transmit array 2
  else
    transmitState = radio.startTransmit(TXarray1, 8);  // transmit array 1

  transmitFlag = true;
  TXarraySelect = !TXarraySelect;  // select other array for next TX
  transmitTimer = millis();  // reset transmit timer
}
