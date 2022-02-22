// -----------------------------------------------------------
// Example_toneGenerator.ino
// Written by Fowler Askew
// AURA Payload Team 2021
// Plays a tone on a single pin for a defined amount of time
// -----------------------------------------------------------

#define TONE_PIN 38
#define FREQ 1000
#define ON_TIME 1000
#define OFF_TIME 1000

void setup() {
  // blink to show that it's going
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(250);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(250);

//  tone(TONE_PIN, FREQ);
//  digitalWrite(13, HIGH);
//  delay(ON_TIME);
}

void loop() {
  // start playing the tone
  // turn on the LED too
  tone(TONE_PIN, FREQ);
  digitalWrite(13, HIGH);
  delay(ON_TIME);

  noTone(TONE_PIN);
  digitalWrite(13, LOW);
  delay(OFF_TIME);
}
