#define PWM_1 5  // PWM pin, motor 1 (7 for EASE board)
#define DIR_1 4 // Direction pin, motor 1 (8 for EASE board)
#define LEDPIN 0
#define interruptPin 2 //Interrupt pin (6 for EASE board)

int debounceTime = 10;  // number of microseconds to wait in the ISR for debouncing
int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 1/*340*/; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 22;  // encoder pulses (rising and falling) for one rotation
int screwPitch = 1;  // TPI of leadscrew
volatile int pulseCount = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  //Serial.begin(115200);
  //delay(10);
  //Serial.println("Turning on motor");
  delay(10);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(DIR_1, HIGH); //Set motor direction forward
  analogWrite(PWM_1, 50); //Turn on motor, low speed
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, CHANGE); //If value of interruptPin changes, run count.
}

void loop() {
  //Serial.println(pulseCount);
  while(pulseCount >= (totalDistance * gearRatio * pulsePerRotate * screwPitch)) //Check to see if fully extended. If so, change value or something. 
  {}
}

void count()
{
  pulseCount++;
  if(pulseCount >= (totalDistance * gearRatio * pulsePerRotate * screwPitch)) //Check to see if fully extended. If so, change value or something. 
  {
    detachInterrupt(digitalPinToInterrupt(interruptPin)); //stop interrupting (this has to happen or the code will never turn the motor off)
    analogWrite(PWM_1, 0); //Turn off motor
    digitalWrite(LEDPIN, LOW);  // turn off LED
  }
  delayMicroseconds(debounceTime);  // waits some amount of time to avoid triggering multiple times for one encoder pulse
}
