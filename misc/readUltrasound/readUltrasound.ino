// -----------------------------------------------------------
// readUltrasound.ino
// Written by Fowler Askew
// AURA Payload Team 2021
// Continually reads the incoming signal from the ultrasonic amplifier
// -----------------------------------------------------------

#define TONE_IN A2  // input pin is A2 (DIO16)
#define sampleDuration 30  // slightly longer than half the input signal period (40us/2 + 10)
#define ultrasoundThreshold 300  // 
unsigned int sampleStart = 0;
int sampleEnd = 0;
int maxInput = 0;
int minInput = 1024;
int currentInput = 0;
int strength = 500;
double smoothingVal = 0.05;

void setup()
{
  pinMode(TONE_IN, INPUT_DISABLE);  // I think this is beneficial on Teensy boards
  //pinMode(TONE_IN, INPUT);  // not strictly necessary, but still good
  analogReadAveraging(1);  // number of samples to average (default 4)
  Serial.begin(115200);
  delay(500);
}

void loop()
{
  maxInput = analogRead(TONE_IN);  // this will be updated to contain the max sample value
  minInput = analogRead(TONE_IN);
  sampleStart = micros();
  while(micros() < sampleStart + sampleDuration)
  {
    currentInput = analogRead(TONE_IN);
    
    if(currentInput > maxInput)  // if current sample is greater than previous max sample
      maxInput = currentInput;  // update max sample
      
    if(currentInput < minInput)  // update min sample
      minInput = currentInput;
  }
  strength = smoothingVal*(maxInput - minInput) + (1-smoothingVal)*strength;
  Serial.println(strength);
  delay(5);  // delay 100 milliseconds to decrease the printing
}
