// -----------------------------------------------------------
// readUltrasound.ino
// Written by Fowler Askew
// AURA Payload Team 2021
// Continually reads the incoming signal from the ultrasonic amplifier
// -----------------------------------------------------------

#define TONE_IN   // input pin is A2 (DIO16)
#define sampleDuration 25  // slightly longer than half the input signal period (40us/2 + 5)
int sampleStart = 0;
int sampleEnd = 0;
int maxInput = 0;
int currentInput = 0;

void setup()
{
  pinMode(TONE_IN, INPUT_DISABLE);  // I think this is beneficial on Teensy boards
  //pinMode(TONE_IN, INPUT);  // not strictly necessary, but still good
  //analogReadAveraging(4);  // number of samples to average (default 4)
  Serial.begin(115200);
  delay(500);
}

void loop()
{
  maxInput = analogRead(TONE_IN);  // this will be updated to contain the max sample value
  sampleStart = micros();
  while(micros() < sampleStart + sampleDuration)
  {
    currentInput = analogRead(TONE_IN);
    
    if(currentInput > maxInput)  // if current sample is greater than previous max sample
      maxInput = currentInput;  // update max sample
  }

  Serial.println(maxInput);
  delay(100);  // delay 100 milliseconds to decrease the printing
}
