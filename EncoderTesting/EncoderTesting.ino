#define PWM_1 5  // PWM pin, motor 1
#define DIR_1 4 // Direction pin, motor 1
#define interruptPin 2 //Interrupt pin

int travelDistance = .125; //inches
int gearRatio = 340; //gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 22;
int pulseCount = 0;
int extended = 0; //0 for false, 1 for true. 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, CHANGE); //If value of interruptPin changes, run count. 
  digitalWrite(DIR_1, HIGH); //Set motor direction forward
  digitalWrite(PWM_1, HIGH); //Turn on motor
}

void loop() {
  // put your main code here, to run repeatedly:
  // If screw has not been fully extended, keep going. Otherwise, stop. 
  if(extended == 1)
  {
    digitalWrite(PWM_1, LOW); //Turn off motor
  }
 
}

void count() {
  pulseCount++;
  if(pulseCount/(gearRatio * pulsePerRotate * 8) >= travelDistance){ //Check to see if fully extended. If so, set extended. 
    extended = 1;
  }
}
