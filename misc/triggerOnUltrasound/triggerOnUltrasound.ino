// -----------------------------------------------------------
// triggerOnUltrasound.ino
// Written by Fowler Askew
// AURA Payload Team 2021
// Indicates when the ultrasound signal is received
// -----------------------------------------------------------

#define TONE_IN 16  // input pin is A2 (DIO16)
#define timeOut 2000000  // amount of time to wait after receiving radio message (max distance (ft) / 1087 (ft/sec)) * 1000000
#define inputThreshold 514  // I think "no signal" will be right around 512ish, 1 count = 3.2mV

int startTime = 0;
int delayTime = 0;
int currentInput = 0;
bool signalDetected = false;

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
  Serial.print("Starting detection period...");
  startTime = micros();
  while(!signalDetected && micros() < startTime + timeOut)
  {
    if(analogRead(TONE_IN) > inputThreshold)
    {
      delayTime = micros() - startTime;  // how long has it been since detection loop started
      signalDetected = true;  // flag the signal detection
    }
  }

  if(signalDetected)
  {
    Serial.print(delayTime);
    Serial.println("us");
  }
  else
    Serial.println("timeout");

  delay(2500);
}
