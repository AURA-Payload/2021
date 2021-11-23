/*
  Customized version of Sparkfun's example code
  I adapted it to use a more accurate class and define all parameters

  Default parameters are:
  Carrier frequency: 915.0 MHz
  Bandwidth: 125.0 kHz
  Spreading factor: 9
  Coding rate: 4/7
  Sync word: private network
  Output power: 10 dBm
  Preamble length: 8 symbols
  Gain: 0(automatic gain control)
  
  RadioLib SX1276 Receive Example using the RFM97C 915MHz

  Gotchas:
    * The RadioLib defaults the SX1276 to 434MHz so the reception is pretty
      poor. This is fixed with a radio.begin(915.0);
    * The RadioLib really requires DIO0 and DIO1 be connected. Reset is optional.
    * Avoid ESP pins 34/35/36/39 as they are inputs only
*/

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib

// radio has the following connections on the subscale board:
int pin_cs = 10;
int pin_dio0 = 39;
int pin_nrst = 31;
int pin_dio1 = 32;
RFM95 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
  Serial.begin(115200);
  delay(200); //Wait for board to be able to print

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
}

void loop() {
  //Serial.print(F("[RFM95] Waiting for incoming transmission ... "));

  byte commands[8];
  int state = radio.receive(commands, 8);

  if (state == ERR_NONE) {
    // packet was successfully received
    Serial.println(F("success!"));

    // print byte 4 of the packet
    Serial.print(F("[RFM95] Data:\t\t\t"));
    Serial.println(commands[4]);

    // print the RSSI
    Serial.print(F("[RFM95] RSSI:\t\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    Serial.print(F("[RFM95] SNR:\t\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    Serial.print(F("[RFM95] Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
}
