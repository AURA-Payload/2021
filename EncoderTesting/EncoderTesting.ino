#define PWM_1 5  // PWM pin, motor 1 (7 for EASE board)
#define DIR_1 4 // Direction pin, motor 1 (8 for EASE board)
#define interruptPin 2 //Interrupt pin (6 for EASE board)

int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 340; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 23;  // encoder pulses (rising and falling) for one rotation
int screwPitch = 1;  // TPI of leadscrew
volatile int pulseCount = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  Serial.begin(115200);
  delay(10);
  Serial.println("Turning on motor");
  delay(10);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING); //If value of interruptPin changes, run count. 
  digitalWrite(DIR_1, HIGH); //Set motor direction forward
  analogWrite(PWM_1, 0); //Turn on motor, low speed
}

void loop() {
  Serial.println(pulseCount);
}

void count()
{
  pulseCount++;
  
  /*if(pulseCount >= (totalDistance * gearRatio * pulsePerRotate * screwPitch)) //Check to see if fully extended. If so, change value or something. 
  {
    detachInterrupt(digitalPinToInterrupt(interruptPin)); //stop interrupting
    analogWrite(PWM_1, 0); //Turn off motor
  }*/
}
