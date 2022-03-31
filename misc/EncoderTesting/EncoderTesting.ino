#define PWM_1 5  // PWM pin, motor 1 (7 for EASE board)
#define DIR_1 4 // Direction pin, motor 1 (8 for EASE board)
#define interruptPin 2 //Interrupt pin (6 for EASE board)
#define outputPin 3  // this pin will toggle in the ISR for debugging purposes

int debounceTime = 5;  // number of microseconds to wait in the ISR for debouncing
int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 340; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 23;  // encoder pulses (rising and falling) for one rotation
int screwPitch = 1;  // TPI of leadscrew
volatile int pulseCount = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(outputPin, OUTPUT);

  while(millis() < 10000)  // this snippet will let us get an idea of how long it takes to change the state of a digital pin
  {
    digitalWrite(outputPin, HIGH);
    digitalWrite(outputPin, LOW);
  }
  
  Serial.begin(115200);
  delay(10);
  Serial.println("Turning on motor");
  delay(10);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING); //If value of interruptPin changes, run count. 
  digitalWrite(DIR_1, HIGH); //Set motor direction forward
  analogWrite(PWM_1, 75); //Turn on motor, low speed
}

void loop() {
  if(pulseCount >= (totalDistance * gearRatio * pulsePerRotate * screwPitch)) //Check to see if fully extended. If so, change value or something. 
  {
    detachInterrupt(digitalPinToInterrupt(interruptPin)); //stop interrupting (this has to happen or the code will never turn the motor off)
    analogWrite(PWM_1, 0); //Turn off motor
  }
}

void count()
{
  digitalWrite(outputPin, HIGH);  // this pin lets us observe the ISR activation on the oscilloscope
  pulseCount++;

  //delayMicroseconds(debounceTime);  // waits some amount of time to avoid triggering multiple times for one encoder pulse
  digitalWrite(outputPin, LOW);
}
