#define BIT0 8
#define BIT1 17
#define BIT2 20
#define BIT3 21

#define SENSE1 A9
#define SENSE2 A8

#include <TimeLib.h>


//byte sensorSelect = 0b00000000;  // this will serve to select the correct light sensor
int sensorValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the value of all 32 sensors
int validTimes[32] = {700, 730, 800, 830, 900, 930, 1000, 1015, 1030, 1045, 1100, 1115, 1130, 1145, 1200, 1215, 1230, 1245, 1300, 1315, 1330, 1345, 1400, 1415, 1430, 1445, 1500, 1515, 1530, 1600, 1630, 1700};
int sensorVals[32] = {00000, 00001, 00010, 00011, 00100, 00101, 00110, 00111, 01000, 01001, 01010, 01011, 01100, 01101, 01110, 01111, 10000, 10001, 10010, 10011, 10100, 10101, 10110, 10111, 11000, 11001, 11010, 11011, 11100, 11101, 11110, 11111};
int sensorHour[] = {7,7,8,8,9,9,10,10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,16,16,17};  // stores the hour associated with each sensor
int sensorMinute[] = {0,30,0,30,0,30,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,45,0,15,30,0,30,0};  // stores the minutes associated with each sensor
int sunCalibration[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the maximum value for each sensor in the sun (preloaded with lowest possible ADC value)
int shadeCalibration[] = {1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,  // stores the minimum value for each sensor in the shade (peloaded with highest possible ADC value
                        1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,
                        1024,1024,1024,1024,1024,1024,1024,1024};
int midCalibration[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the middle value for each sensor


void setup() {
  Serial.begin(115200);
  pinMode(BIT0, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT3, OUTPUT);
  pinMode(SENSE1, INPUT);
  pinMode(SENSE2, INPUT);

  updateSelection(0);  // writes 0000 to sensor select pins

  delay(250);

  while(!Serial){} // wait until the serial port is ready

  Serial.println("Place sundial in sunlight and press any key to calibrate");  // prompt user to calibrate for sunlight
  while(Serial.available() == 0)  // wait until something is typed
  
  calibrateSun();  // run the sunlight calibration
  Serial.println("Sunlight calibrated");  // indicate that sun calibration is done
  printArray(sunCalibration, 0, 31);  // print the calibration array
  
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(10);
  }

  delay(250);
  
  Serial.println("Place sundial in shade and press any key to calibrate");
  while(Serial.available() == 0)
  
  calibrateShade();  // run the shade calibration
  Serial.println("Shade calibrated");  // indicate that shade calibration is done
  printArray(shadeCalibration, 0, 31);  // print the calibration array

  
  
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(10);
  }

  for(int i = 0; i < 32; i++){
    midCalibration[i] = ((sunCalibration[i] + shadeCalibration[i])/2);
  }

  setSyncProvider(getTeensy3Time);  // I believe this syncs both RTCs on the Teensy
  
  Serial.begin(115200);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  
  delay(100);
  if (timeStatus()!= timeSet)
    Serial.println("Unable to sync with the RTC");
    
  else
    Serial.println("RTC has set the system time");
}

void loop()
{
  int hours = hour();
  int minutes = minute();
  int timeVal = (hours * 100) + minutes;
  int sensor = findTime(timeVal);
  int found = 0;
  int sensorValue = -1;
  while(sensor > -1 && found == 0) {
    sensorValue = readSensor(sensor);
    if(sensorValue == midCalibration[sensor]){
      found = 1;
    }
    Serial.println("Rotate...");
  }
  delay(100);
  Serial.println("Found!");
  }

void readSensors()  // reads all the sensors into the sensorValues array
{
  for(byte readLoop = 0; readLoop < 16; readLoop++)
  {
    updateSelection(readLoop);
    delayMicroseconds(200);  // this delay allows the analog signal to propage before being read (see POLO trello card for 'scope measurement)
    sensorValues[readLoop] = analogRead(SENSE1);  // read the values into an array
    sensorValues[readLoop + 16] = analogRead(SENSE2);
  }
}

int readSensor(byte senseIndex)  // returns the value of a specific sensor
{
  updateSelection(senseIndex);
  delayMicroseconds(200);  // this delay allows the analog signal to propage before being read (see POLO trello card for 'scope measurement)
  
  if(senseIndex & 0b00010000)  // if the index is in the 16-32 range
    return analogRead(SENSE2);
  else
    return analogRead(SENSE1);
}

void printArray(int arrayIn[], int printStart, int printEnd)  // print an array Use the array indexes you want (0-31 will print a 32 value array)
{
  for(byte printLoop = printStart; printLoop <= printEnd; printLoop++)
  {
    Serial.print(arrayIn[printLoop]);
    Serial.print("\t");
  }
  Serial.print('\n');
}

void updateSelection(byte selectVar) {
  bool bitVal = selectVar & 0b00000001; // set LSB
  digitalWrite(BIT0, bitVal);
  bitVal = selectVar & 0b00000010;  // set bit 1
  digitalWrite(BIT1, bitVal);
  bitVal = selectVar & 0b00000100;  // set bit 2
  digitalWrite(BIT2, bitVal);
  bitVal = selectVar & 0b00001000;  // set MSB
  digitalWrite(BIT3, bitVal);
}

void calibrateSun()
{
  for(byte calCycles = 0; calCycles < 5; calCycles++)  // check the sensors 5 times for the highest value available
  {
    for(byte readLoop = 0; readLoop < 16; readLoop++)  // read through all 16 sensor channels
    {
      updateSelection(readLoop);  // select sensor
      delayMicroseconds(200);  // allow values to stabilize
      
      if(analogRead(SENSE1) > sunCalibration[readLoop])  // if current value is larger than past value
        sunCalibration[readLoop] = analogRead(SENSE1);  // store current value in place of past value
        
      if(analogRead(SENSE2) > sunCalibration[readLoop + 16])  // repeat for other input
        sunCalibration[readLoop + 16] = analogRead(SENSE2);
    }
    delay(10);  // wait to see if values fluctuate
  }
}

void calibrateShade()
{
  for(byte calCycles = 0; calCycles < 5; calCycles++)  // check the sensors 5 times for the lowest value available
  {
    for(byte readLoop = 0; readLoop < 16; readLoop++)  // read through all 16 sensor channels
    {
      updateSelection(readLoop);  // select sensor
      delayMicroseconds(200);  // allow values to stabilize
      
      if(analogRead(SENSE1) < shadeCalibration[readLoop])  // if current value is smaller than past value
        shadeCalibration[readLoop] = analogRead(SENSE1);  // store current value in place of past value
        
      if(analogRead(SENSE2) < shadeCalibration[readLoop + 16])  // repeat for other input
        shadeCalibration[readLoop + 16] = analogRead(SENSE2);
    }
    delay(10);  // wait to see if values fluctuate
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

int findTime(int timeVal){
  for(int i = 0; i < 32; i++){
    if(timeVal == validTimes[i]){
      return i; 
    }
  }
  return -1; 
}
