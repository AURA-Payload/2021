#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float levelValue = 1.18;  // target value for level
float levelTolerance = 0.04;  // acceptable range for "level"

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
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" m/s^2\t");

  Serial.print("Error: ");
  Serial.print(levelValue - event.acceleration.y);
  
  if((levelValue - levelTolerance) < event.acceleration.y && event.acceleration.y < (levelValue + levelTolerance))
    Serial.println("\tLevel :)");
  else
    Serial.println("\tNot level >:(");

  delay(250);
}
