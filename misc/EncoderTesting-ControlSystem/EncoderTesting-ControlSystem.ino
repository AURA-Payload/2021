// Encoder test code with control system
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_1 5
#define DIR_1 4

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
unsigned long prevT = 0;
double eprev = 0;
double eintegral = 0;

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
  
  Serial.println("target pos");
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
  double kd = 0.0;
  double ki = 0.0;

  // time difference
  unsigned long currT = micros();
  double deltaT = (double)((currT - prevT)*(0.000001));
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  double dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  double u = kp*e + kd*dedt + ki*eintegral;

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

  // store previous error
  eprev = e;

  if(millis() >= printTime + printerval)
  {
    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    /*Serial.print(" ");
    Serial.print(deltaT);
    Serial.print(" ");
    Serial.print(dedt);*/
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
