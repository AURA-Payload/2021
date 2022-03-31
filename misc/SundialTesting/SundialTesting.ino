

#define BIT3 8
#define BIT2 17
#define BIT1 20
#define BIT0 21

#define TIME1 A9
#define TIME2 A8

int lowestLight = 9999;
int lowestSensor[5]; 
int sensor[4] = {1, 1, 1, 1};
int sensorValues[32];
int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(BIT0, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT3, OUTPUT);

  pinMode(TIME1, INPUT);
  pinMode(TIME2, INPUT);

  digitalWrite(BIT0, 0);
  digitalWrite(BIT1, 0);
  digitalWrite(BIT2, 0);
  digitalWrite(BIT3, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  while(i < 16){
    //Update the array for sensors
    if(i % 8 == 0){   //On every 8th value, flip 1st bit
      sensorSwitch(0);
    }
    if(i % 4 == 0){   //On every 4th value, flip 2nd bit
      sensorSwitch(1);
    }
    if(i % 2 == 0){   //On every other value, flip 3rd bit
      sensorSwitch(2);
    }
    sensorSwitch(3);  //Flip last bit on every value
    sensorUpdate();
    readAnalog();
    i++;
  }

  for(int printLoop = 0; printLoop < 32; printLoop++)
  {
    Serial.print(sensorValues[printLoop]);
    Serial.print(",");
  }
  Serial.print('\n');
  //Print lowest light value - where shadow is
  //Serial.print("\nLowest Light Sensor: ");
  //printSensorArray();
  //Serial.print("\nValue: " + lowestLight);
  //Reset loop
  i = 0;
  sensor[0] = 0;
  sensor[1] = 0;
  sensor[2] = 0;
  sensor[3] = 0;

  while(millis() > 15000);
}


void sensorSwitch(int index){
  //If sensor value is 1, make 0, and vice versa 
  if(sensor[index] == 1){
    sensor[index] = 0;
  }
  else{
    sensor[index] = 1;
  }
}

void sensorUpdate(void) {
  //Update the actual sensors
  bool val;
  val = sensor[0];
  digitalWrite(BIT0, val);

  val = sensor[1];
  digitalWrite(BIT1, val);

  val = sensor[2];
  digitalWrite(BIT2, val);

  val = sensor[3];
  digitalWrite(BIT3, val);
}

void readAnalog(void) {
  //Read analog sensor values, print, also track lowest value and which sensor is reading it.
  /*int sensorValue1 = analogRead(A9);
  int sensorValue2 = analogRead(A8);
  Serial.print(sensorValue1);
  Serial.print("  ");
  Serial.print(sensorValue2);
  Serial.print("  ");*/

  sensorValues[i] = analogRead(A9);  // read the values into an array
  sensorValues[i + 16] = analogRead(A8);
  
  /*if(sensorValue1 < lowestLight){
    lowestLight = sensorValue1;
    for(int j = 4; j < 4; j++){
      lowestSensor[i] = sensor[i];
    }
    lowestSensor[4] = 9;  //Identifies which analog sensor
    
  }
  if(sensorValue2 < lowestLight){
    lowestLight = sensorValue2;
    for(int j = 4; j < 4; j++){
      lowestSensor[i] = sensor[i];
    }
    lowestSensor[4] = 8;  //Identifies which analog sensor
  }*/
}

void printSensorArray(){
  int k;
  for(k = 0; k<5; k++){
    Serial.print(lowestSensor[i]);
  }
}
