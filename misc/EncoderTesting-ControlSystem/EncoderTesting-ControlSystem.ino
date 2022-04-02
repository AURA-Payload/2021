// Encoder test code with control system - derivative and integral terms removed
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_1 5
#define DIR_1 4

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
double eprev = 0;

int target = 0;
int printerval = 5;  // millisecond interval to print values
int targetInterval = 1000;  // 2.5 seconds between switching targets
unsigned long printTime = 0;
unsigned long targetSwitch = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  
  delay(2000);
}

void loop() {

  if(millis() >= targetSwitch + targetInterval)
  {
    /*if(target == 20)
      target = 15;
    else
      target = 20;*/
    target += 7;

    targetSwitch = millis();
  }

  //target = 1000*sin(prevT/1e6);

  // PID constants
  double kp = 15;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // control signal
  double u = kp*e;

  // motor power
  double pwr = fabs(double(u));
  if(pwr > 255){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
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
  int chB = digitalRead(ENCB);
  if(chB > 0){
    posi++;
  }
  else{
    posi--;
  }
}
