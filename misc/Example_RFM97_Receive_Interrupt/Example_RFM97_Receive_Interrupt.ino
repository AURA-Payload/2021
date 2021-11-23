/*
   Customized version of Radiolib receive with interrupts

   This example listens for LoRa transmissions and tries to
   receive them. Once a packet is received, an interrupt is
   triggered.

   Default parameters are:
    Carrier frequency: 915.0 MHz
    Bandwidth: 125.0 kHz
    Spreading factor: 9
    Coding rate: 4/7
    Sync word: private network
    Output power: 10 dBm
    Preamble length: 8 symbols
    Gain: 0(automatic gain control)

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// RFM95 has the following connections:
int pin_cs = 10;
int pin_dio0 = 39;
int pin_nrst = 31;
int pin_dio1 = 32;
RFM95 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//RFM95 radio = RadioShield.ModuleA;

void setup() {
  Serial.begin(115200);

  // initialize RFM95 with default settings
  Serial.print(F("[RFM95] Initializing ... "));
  int state = radio.begin(915.0,  // carrier freq (MHz)
                          125.0,  // bandwidth (kHz)
                          9,  // spreading factor
                          7,  // coding rate
                          0x12,  // sync word
                          15,  // output power (dBm)
                          8,  // preamble length (symbols)
                          0  // gain (0 is automatic control)
                          );
  radio.setCRC(true);  // enables cyclic redundancy check
  
  if (state == ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[RFM95] Starting to listen ... "));
  state = radio.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

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

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    //int state = radio.readData(str);

    // you can also read received data as byte array
      byte received[8];
      int state = radio.readData(received, 8);

    if (state == ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[RFM95] Received packet!"));

      // print data of the packet
      Serial.print(F("[RFM95] Data:\t\t"));
      Serial.print(received[0]);
      Serial.print("\t");
      Serial.print(received[1]);
      Serial.print("\t");
      Serial.print(received[2]);
      Serial.print("\t");
      Serial.print(received[3]);
      Serial.print("\t");
      Serial.print(received[4]);
      Serial.print("\t");
      Serial.print(received[5]);
      Serial.print("\t");
      Serial.print(received[6]);
      Serial.print("\t");
      Serial.println(received[7]);

//      // print RSSI (Received Signal Strength Indicator)
//      Serial.print(F("[RFM95] RSSI:\t\t"));
//      Serial.print(radio.getRSSI());
//      Serial.println(F(" dBm"));
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

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[RFM95] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[RFM95] Failed, code "));
      Serial.println(state);

    }

    // put module back to listen mode
    radio.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

}
