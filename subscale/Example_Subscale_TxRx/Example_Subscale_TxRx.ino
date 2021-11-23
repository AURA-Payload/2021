/*
   Test code for subscale board using Radiolib's SX127x_Transmit_Interrupt and SX127x_Receive_Interrupt

   Listens for 8 byte arrays and prints them to Serial
   Transmits 8 byte arrays periodically

   Default parameters are:
    Carrier frequency: 915.0 MHz
    Bandwidth: 125.0 kHz
    Spreading factor: 9
    Coding rate: 4/7
    Sync word: 0x12
    Output power: 10 dBm
    Preamble length: 8 symbols
    Gain: 0(automatic gain control)

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// RFM95 has the following connections:
#define CSPIN 10
#define DIO0PIN 39
#define NRSTPIN 31
#define DIO1PIN 32

RFM95 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);

// Transmit variables
byte TXarray1[] = {0xFF, 0xFD, 0xFB, 0xF9, 0xF7, 0xF5, 0xF3, 0xF1};
byte TXarray2[] = {0xFE, 0xFC, 0xFA, 0xF8, 0xF6, 0xF4, 0xF2, 0xF0};
byte TXarraySelect = 0;  // 0 is TXarray1, else TXarray2
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 2000;  // time between tranmissions

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
  delay(500);

  // initialize RFM95 with default settings
  Serial.print(F("[RFM95] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                          125.0,  // bandwidth (kHz)
                          9,  // spreading factor
                          7,  // coding rate
                          0x12,  // sync word
                          15,  // output power (dBm)
                          8,  // preamble length (symbols)
                          0  // gain (0 is automatic control)
                          );
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

  radio.setDio0Action(setFlag);  // function that will be called when something is done

  // start listening for LoRa packets
  //Serial.println(F("[RFM95] Starting to listen ... "));
  receiveState = radio.startReceive();

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
  // radio.scanChannel();
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
        Serial.println(F("[RFM95] Received packet!"));
  
        // print data of the packet
        Serial.print(F("[RFM95] Data:\t\t"));
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
        Serial.print(F("\t[RFM95] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
  //
  //      // print SNR (Signal-to-Noise Ratio)
  //      Serial.print(F("[RFM95] SNR:\t\t"));
  //      Serial.print(radio.getSNR());
  //      Serial.println(F(" dB"));
  //
  //      // print frequency error
  //      Serial.print(F("[RFM95] Frequency error:\t"));
  //      Serial.print(radio.getFrequencyError());
  //      Serial.println(F(" Hz"));
      }
      else if (receiveState == ERR_CRC_MISMATCH)  // packet received malformed
        Serial.println(F("[RFM95] CRC error!"));
      else  // some other error
      {
        Serial.print(F("[RFM95] Failed, code "));
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
  Serial.println(F("[RFM95] Sending packet ... "));
  if(TXarraySelect)  // choose array to transmit based on TXarraySelect
    transmitState = radio.startTransmit(TXarray2, 8);  // transmit array 2
  else
    transmitState = radio.startTransmit(TXarray1, 8);  // transmit array 1

  transmitFlag = true;
  TXarraySelect = !TXarraySelect;  // select other array for next TX
  transmitTimer = millis();  // reset transmit timer
}
