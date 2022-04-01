#define BIT0 8
#define BIT1 17
#define BIT2 20
#define BIT3 21

#define SENSE1 A9
#define SENSE2 A8

//byte sensorSelect = 0b00000000;  // this will serve to select the correct light sensor
int sensorValues = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the values of all 32 sensors
int sunCalibration = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // stores the maximum value for each sensor in the sun (preloaded with lowest possible ADC value)
int shadeCalibration = {1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,  // stores the minimum value for each sensor in the shade (peloaded with highest possible ADC value)
                        1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,
                        1024,1024,1024,1024,1024,1024,1024,1024};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(BIT0, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT3, OUTPUT);

  pinMode(SENSE1, INPUT);
  pinMode(SENSE2, INPUT);

  updateSelection(0);  // writes 0000 to sensor select pins

  delay(250);

  Serial.println("Place sundial in sunlight and press any key to calibrate");  // prompt user to calibrate for sunlight
  while(Serial.available() == 0)  // wait until something is typed
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(2);
  }
  calibrateSun();  // run the sunlight calibration
  Serial.println("Sunlight calibrated");  // indicate that sun calibration is done
  printArray(sunCalibration, 0, 32);  // print the calibration array

  delay(250);
  
  Serial.println("Place sundial in shade and press any key to calibrate");
  while(Serial.available() == 0)
  while(Serial.available() > 0)  // get all the incoming characters out of the serial buffer
  {
    Serial.read();
    delay(2);
  }
  calibrateShade();  // run the shade calibration
  Serial.println("Shade calibrated");  // indicate that shade calibration is done
  printArray(shadeCalibration, 0, 32);  // print the calibration array
}

void loop()
{
  readSensors();

  printArray(sensorValues, 0, 32);
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

void printArray(int arrayIn[], int printStart, int printEnd)  // print an array
{
  for(byte printLoop = printStart; printLoop < printEnd; printLoop++)
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
