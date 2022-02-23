/*
   Test code for receiving FSK tone outputs from Example_RF_Tone.ino
*/

// include the library
#include <RadioLib.h>

// RFM95 has the following connections:
#define CSPIN 10
#define DIO0PIN 39
#define NRSTPIN 31
#define DIO1PIN 32
#define DIO2PIN 30

SX1278 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);
int state = 0;

void setup()
{
  Serial.begin(9600);
  delay(500);

  // initialize RFM95 with default settings
  Serial.print(F("[RFM95] Initializing FSK ... "));
  state = radio.beginFSK();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  delay(250);
  
  if (state == RADIOLIB_ERR_NONE)  // if radio initialized correctly
    Serial.println(F("init success!"));
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(state);
    while (true);  // stop the program
  }

  state = radio.setFrequency(905.0);  // frequency can be set to something that doesn't have much traffic
  state = radio.setBitRate(100.0);
  state = radio.setFrequencyDeviation(10.0);
  state = radio.setRxBandwidth(10.0);
  state = radio.setOutputPower(1.0);
  state = radio.setDataShaping(0.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to set configuration, code "));
    Serial.println(state);
    while (true);
  }

  state = radio.receiveDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Unable to start direct reception mode, code "));
    Serial.println(state);
  }
}

void loop()
{
  int duration = pulseIn(DIO2PIN, HIGH);
  //Serial.println(duration);
}
