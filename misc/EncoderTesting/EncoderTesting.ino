#define PWM_1 5  // PWM pin, motor 1 (7 for EASE board)
#define DIR_1 4 // Direction pin, motor 1 (8 for EASE board)
#define LEDPIN 0
#define ENCA 3 //Interrupt pin (6 for EASE board)
#define ENCB 2

int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 300; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 7;  // encoder pulses (rising and falling) for one rotation
int screwPitch = 1;  // TPI of leadscrew
int target = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // sets the target to hit (should be 1 shaft rotation)
volatile int pulseCount = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  Serial.begin(115200);
  delay(10);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(DIR_1, LOW); //Set motor direction reverse
  analogWrite(PWM_1, 255); //Turn on motor, low speed
  attachInterrupt(digitalPinToInterrupt(ENCA), count, RISING); //If value of interruptPin changes, run count.
}

void loop() {
  Serial.print(target);
  Serial.print(" ");
  Serial.println(pulseCount);
  delay(25);
//  while(pulseCount >= (totalDistance * gearRatio * pulsePerRotate * screwPitch)) //Check to see if fully extended. If so, change value or something. 
//  {}
}

void count()
{
  if(digitalRead(ENCB))
    pulseCount++;
  else
    pulseCount--;
    
  if(pulseCount >= target) //Check to see if fully extended. If so, change value or something. 
  {
    analogWrite(PWM_1, 0); //Turn off motor
    digitalWrite(LEDPIN, LOW);  // turn off LED
  }
}
