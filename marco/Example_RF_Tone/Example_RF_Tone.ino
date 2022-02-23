/*
   Example_RF_Tone.ino

   Code adapted from RadioLib SX127x FSK Modem Example

   Illustrates features of the FSK mode
   Specifically shows how to use the transmitDirect() function of the radio

   This is written for the MARCO prototype built on perfboard
*/

// include the library
#include <RadioLib.h>

// These are the connections used on my ground station board:
#define CSPIN 10
#define DIO0PIN 34
#define NRSTPIN 33
#define DIO1PIN 35
#define RXENPIN 36
#define TXENPIN 37
#define DIO2PIN 38

int state = 0;
SX1276 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);

void setup() {
  Serial.begin(9600);
  radio.setRfSwitchPins(RXENPIN, TXENPIN);  // set up RF switch pins

  // initialize SX1276 FSK modem with default settings
  Serial.print(F("[SX1276] Initializing FSK ... "));
  state = radio.beginFSK();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  
  delay(250);

  // if needed, you can switch between LoRa and FSK modes
  //
  // radio.begin()       start LoRa mode (and disable FSK)
  // radio.beginFSK()    start FSK mode (and disable LoRa)

  // the following settings can also
  // be modified at run-time
  state = radio.setFrequency(905.0);  // frequency can be set to something that doesn't have much traffic
  state = radio.setBitRate(100.0);
  state = radio.setFrequencyDeviation(10.0);
  state = radio.setRxBandwidth(250.0);
  state = radio.setOutputPower(1.0);
  state = radio.setDataShaping(0.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to set configuration, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  // FSK modem supports direct data transmission
  // in this mode, SX127x directly transmits any data
  // sent to DIO1 (data) and DIO2 (clock)

  // activate direct mode transmitter
  state = radio.transmitDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1276] Unable to start direct transmission mode, code "));
    Serial.println(state);
  }
  else
    Serial.println("transmitting");

  // using the direct mode, it is possible to transmit
  // FM notes with Arduino tone() function

  // it is recommended to set data shaping to 0
  // (no shaping) when transmitting audio

  // transmit FM tone at 1000 Hz for 1 second, then 500 Hz for 1 second
  tone(DIO2PIN, 1000);
  delay(1000);
  tone(DIO2PIN, 500);
  delay(1000);
  noTone(DIO2PIN);
  
  /*digitalWrite(DIO2PIN, HIGH);
  delay(2000);
  while(true);
  {}
  digitalWrite(DIO2PIN, LOW);*/

  radio.standby();

  // NOTE: after calling transmitDirect(), SX127x will start
  // transmitting immediately! This signal can jam other
  // devices at the same frequency, it is up to the user
  // to disable it with standby() method!

  // direct mode transmissions can also be received
  // as bit stream on DIO1 (data) and DIO2 (clock)
  /*state = radio.receiveDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1276] Unable to start direct reception mode, code "));
    Serial.println(state);
  }*/
  delay(2000);

  // NOTE: you will not be able to send or receive packets
  // while direct mode is active! to deactivate it, call method
  // radio.packetMode()
}
