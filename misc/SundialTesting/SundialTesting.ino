#define BIT0 8
#define BIT1 17
#define BIT2 20
#define BIT3 21

#define SENSE1 A9
#define SENSE2 A8

//byte sensorSelect = 0b00000000;  // this will serve to select the correct light sensor
int sensorValues[32];  // stores the values of all 32 sensors
int sunCalibration[32];  // stores the maximum value for each sensor in the sun
int shadeCalibration[32];  // stores the minimum value for each sensor in the shade

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
