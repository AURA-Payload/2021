#define BIT0 8
#define BIT1 17
#define BIT2 20
#define BIT3 21

#define TIME1 A9
#define TIME2 A8

int lowestLight = 9999;
int lowestSensor[5];
byte sensorSelect = 0b00000000;  // this will serve to select the correct light sensor
bool val;
//int sensorSelect[4] = {0, 0, 0, 0};  // THIS IS SET UP TO BE LSB TO MSB - THIS MEANS sensorSelect[0] = BIT0 = LSB
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

  
  /*digitalWrite(BIT0, sensorSelect[0]);
  digitalWrite(BIT1, sensorSelect[1]);
  digitalWrite(BIT2, sensorSelect[2]);
  digitalWrite(BIT3, sensorSelect[3]);*/

}

void loop() {
  // put your main code here, to run repeatedly:
  /*while(i < 16){
    //Update the array for sensors
    if(i % 8 == 0){   //On every 8th cycle, flip MSB
      sensorSwitch(3);
    }
    if(i % 4 == 0){   //On every 4th cycle, flip bit 2
      sensorSwitch(2);
    }
    if(i % 2 == 0){   //On every other cycle, flip bit 1
      sensorSwitch(1);
    }
    sensorSwitch(0);  //Flip LSB on every cycle
    sensorUpdate();
    readAnalog();
    i++;
  }*/

  while(sensorSelect < 16){
    //Update the array for sensors
    sensorUpdate();
    readAnalog();
    sensorSelect++;
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
  /*i = 0;
  sensorSelect[0] = 0;
  sensorSelect[1] = 0;
  sensorSelect[2] = 0;
  sensorSelect[3] = 0;*/
  sensorSelect = 0;
}


/*void sensorSwitch(int index){
  //If sensor value is 1, make 0, and vice versa 
  if(sensorSelect[index] == 1){
    sensorSelect[index] = 0;
  }
  else{
    sensorSelect[index] = 1;
  }
}*/

void sensorUpdate(void) {
  val = sensorSelect & 0b00000001; // set LSB
  digitalWrite(BIT0, val);
  val = sensorSelect & 0b00000010;  // set bit 1
  digitalWrite(BIT1, val);
  val = sensorSelect & 0b00000100;  // set bit 2
  digitalWrite(BIT2, val);
  val = sensorSelect & 0b00001000;  // set MSB
  digitalWrite(BIT3, val);
  
  //Update the actual sensors
  /*val = sensorSelect[0];
  digitalWrite(BIT0, val);

  val = sensorSelect[1];
  digitalWrite(BIT1, val);

  val = sensorSelect[2];
  digitalWrite(BIT2, val);

  val = sensorSelect[3];
  digitalWrite(BIT3, val);*/
}

void readAnalog(void) {
  //Read analog sensor values, print, also track lowest value and which sensor is reading it.
  /*int sensorValue1 = analogRead(A9);
  int sensorValue2 = analogRead(A8);
  Serial.print(sensorValue1);
  Serial.print("  ");
  Serial.print(sensorValue2);
  Serial.print("  ");*/

  //sensorValues[i] = analogRead(A8);  // read the values into an array
  //sensorValues[i + 16] = analogRead(A9);

  sensorValues[sensorSelect] = analogRead(A9);  // read the values into an array
  sensorValues[sensorSelect + 16] = analogRead(A8);
  
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
