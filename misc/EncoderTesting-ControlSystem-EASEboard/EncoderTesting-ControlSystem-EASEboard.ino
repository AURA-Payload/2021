// Encoder test code with control system - derivative and integral terms removed
// This code uses the PCINT function on DIO17 (PCINT11)
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 17 // interrupt on pin 17 (PC3)
#define ENCB 16
#define PWM_1 9
#define DIR_1 8

#define totalDistance 17L //number of motor shaft rotations to complete
#define gearRatio 326L // for testing output shaft accuracy: 302 for POLO, 349 for EASE
//#define gearRatio 1L  // for testing small distance accuracy
#define pulsePerRotate 11L  // encoder pulses (rising) for one rotation: 7 for POLO, 11 for EASE
#define screwPitch 8L  // TPI of leadscrew

volatile long posi = 0; // specify posi as volatile

long kp = 1;
long pos = 0;
long e = 0;
long u = 0;
long pwr = 0;
int dir = 1;

long target = 0;

int printerval = 10;  // millisecond interval to print values
unsigned long printTime = 0;  // timer for printing stuff

void setup() {
  target = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // sets the target
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  digitalWrite(PWM_1,LOW);
  digitalWrite(DIR_1,LOW);

  PCICR |= 0b00000010;  // enable interrupts on PC register
  PCMSK1 |= 0b00001000;  // use interrupt mask on D17/A3/PC3

  delay(1000);
  Serial.println(target);
  delay(1000);
}

void loop() {  
  // error
  e = posi - target;

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

  if(millis()-printTime >= printerval)
  {
    Serial.print(target);
    Serial.print(" ");
    Serial.print(posi);
    Serial.print(" ");
    //Serial.print(u);
    //Serial.print(" ");
    //Serial.print(pwr);
    //Serial.print(" ");
    //Serial.print(dir);
    Serial.println();

    printTime = millis();
  }
}

void setMotor(int dir, int pwmVal, int pwmPin, int dirPin){
  analogWrite(pwmPin,pwmVal);
  
  if(dir > 0)
    digitalWrite(dirPin,HIGH);
  else
    digitalWrite(dirPin,LOW);
}

ISR (PCINT1_vect){  // encoder read function
  if(PINC & 0b00001000)  // if interrupt is high (rising signal)
  {
    if(PINC & 0b00000100)  // if PC2 (DIO16) is high
      posi++;  // increment position
    
    else  // is PC2 is low
      posi--;  // decrement position
  }
}
