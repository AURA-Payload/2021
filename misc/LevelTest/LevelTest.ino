#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define PWM_A 6  // PWM pin, motor A (SOAR)
#define DIR_A 5  // Direction pin, motor A

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float levelValue = 1.18;  // target value for level
float levelTolerance = 0.1;  // acceptable range for "level"
float errorTerm = 0;
float pGain = 30;
int motorValue = 0;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  delay(250);

  while(!Serial){}
  
  Serial.println("Welcome to the levelling test! Have a wonderful day!");

  if(accel.begin())
  accel.setRange(ADXL345_RANGE_2_G);
  Serial.println("ADXL initialized, 1G");
}

void loop() 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("Y: ");
//  Serial.print(event.acceleration.y);
//  Serial.print(" m/s^2\t");

  errorTerm = levelValue - event.acceleration.y;
  motorValue = errorTerm * pGain;
  motorValue = constrain(motorValue, -255, 255);

  //Serial.print("MotorVal: ");
  Serial.println(motorValue);
  
//  if((levelValue - levelTolerance) < event.acceleration.y && event.acceleration.y < (levelValue + levelTolerance))
//    Serial.println("\tLevel :)");
//  else
//    Serial.println("\tNot level >:(");

//  if(abs(errorTerm) < levelTolerance)
//    Serial.println("\tLevel :)");
//  else
//    Serial.println("\tNot level >:(");

  setMotorA(motorValue);

  delay(25);
}

void setMotorA(int speedVal)  // speedVal between -255 and 255
{
  if(speedVal > 0)  // speed is positive
    digitalWrite(DIR_A, HIGH);
  else
    digitalWrite(DIR_A, LOW);
    
  analogWrite(PWM_A, abs(speedVal));
}
