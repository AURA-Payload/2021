// Encoder test code with control system - derivative and integral terms removed
// This code uses the PCINT function on DIO17 (PCINT11)
// Based on code from Curio Res: https://github.com/curiores/ArduinoTutorials
// Watch this video for more information: https://www.youtube.com/watch?v=dTGITLnYAY0

#define ENCA 17 // interrupt on pin 17 (PC3)
#define ENCB 16
#define PWM_1 9
#define DIR_1 8

#define totalDistance 17 //number of motor shaft rotations to complete
#define gearRatio 326 // for testing output shaft accuracy: 302 for POLO, 349 for EASE
//#define gearRatio 1  // for testing small distance accuracy
#define pulsePerRotate 11  // encoder pulses (rising) for one rotation: 7 for POLO, 11 for EASE
#define screwPitch 8  // TPI of leadscrew

volatile unsigned int posi = 0; // specify posi as volatile
unsigned int target = totalDistance * gearRatio * pulsePerRotate * screwPitch;  // how many encoder pulses to travel

int printerval = 10;  // millisecond interval to print values
unsigned long printTime = 0;  // timer for printing stuff

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  pinMode(PWM_1,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  digitalWrite(PWM_1,LOW);  // don't let the motor rotate at startup
  digitalWrite(DIR_1,LOW);

  PCICR |= 0b00000010;  // enable interrupts on PC register
  PCMSK1 |= 0b00001000;  // use interrupt mask on D17/A3/PC3

  delay(1000);
  Serial.println(target);  // print how far we're going
  delay(1000);
}

void loop() {  
  if(posi < target)  // we haven't reached the target distance
    setMotor(255);  // run the motor
  else  // if we have reached the target
    setMotor(0);  // stop the motor

  if(millis()-printTime >= printerval)
  {
    Serial.print(target);
    Serial.print(" ");
    Serial.print(posi);
    Serial.println();

    printTime = millis();
  }
}

void setMotor(int pwmVal)
{
  analogWrite(PWM_1,abs(pwmVal));
  
  if(pwmVal > 0)
    digitalWrite(DIR_1,HIGH);
  else
    digitalWrite(DIR_1,LOW);
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
