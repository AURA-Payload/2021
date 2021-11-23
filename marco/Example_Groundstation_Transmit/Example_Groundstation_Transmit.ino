/*
  Customized version of Sparkfun's example code
  I adapted it to define all parameters and output a byte array

  Default parameters are:
  Carrier frequency: 434.0 MHz
  Bandwidth: 125.0 kHz
  Spreading factor: 9
  Coding rate: 4/7
  Sync word: private network
  Output power: 10 dBm
  Preamble length: 8 symbols
  Gain: 0(automatic gain control)
  
  RadioLib SX1276 Transmit Example using the 1W LoRa Module @ 915MHz

  Gotchas:
      The RadioLib defaults the SX1276 to 434MHz so the reception is pretty
      poor. This is fixed with a radio.begin(915.0);
      The RadioLib really requires DIO0 and DIO1 be connected. Reset is optional.
      Avoid ESP pins 34/35/36/39 as they are inputs only
*/

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib

// These are the connections used on my ground station board:
int pin_cs = 10;
int pin_dio0 = 34;
int pin_nrst = 33;
int pin_dio1 = 35;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
  Serial.begin(115200);
  delay(500); //Wait for board to be able to print

  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(915.0,  // carrier freq (MHz)
                          125.0,  // bandwidth (kHz)
                          9,  // spreading factor
                          7,  // coding rate
                          0x12,  // sync word
                          15,  // output power (dBm)
                          8,  // preample length (symbols)
                          0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check
  
  if (state == ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  int pin_rx_enable = 36;
  int pin_tx_enable = 37;
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
}

int counter = 0;

void loop() {
  Serial.print(F("[SX1276] Transmitting packet ... "));

  // you can also transmit byte array up to 256 bytes long
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("[SX1276] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // wait before transmitting again
  delay(2000);
}
