// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM_1 5
#define DIR_1 4

volatile int posi = 0; // specify posi as volatile

int kp = 25;
int pos = 0;
int e = 0;
int u = 0;
int pwr = 0;
int dir = 1;

int totalDistance = 1; //number of motor shaft rotations to complete
int gearRatio = 300; // gearRatio * rotationsOfScrew = rotationsOfMotor
int pulsePerRotate = 7;  // encoder pulses (rising and falling) for one rotation
int screwPitch = 1;  // TPI of leadscrew
int target = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // sets the target to hit (should be 1 shaft rotation)

int printerval = 10;  // millisecond interval to print values
unsigned long printTime = 0;  // timer for printing stuff

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);

  Serial.println("Getting ready");
  delay(2000);
}

void loop() {
  if(Serial.available())  // reset position when serial received
  {
    Serial.read();
    posi = 0;
  }

  // Read the position
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  e = pos - target;

  // control signal
  u = kp*e;

  // motor power
  pwr = abs(u);
  if(pwr > 255){
    pwr = 255;
  }

  // motor direction
  dir = 1;
  if(u < 0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM_1,DIR_1);

  if(millis() >= printTime + printerval)
  {
    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(pwr);
    Serial.println();

    printTime = millis();
  }
  delayMicroseconds(10);
}

void setMotor(int dir, int pwmVal, int pwmPin, int dirPin){
  analogWrite(pwmPin,pwmVal);
  
  if(dir > 0)
    digitalWrite(dirPin,HIGH);
  else
    digitalWrite(dirPin,LOW);
}

void readEncoder(){
  if(digitalRead(ENCB) > 0)
    posi++;
    
  else
    posi--;
}
