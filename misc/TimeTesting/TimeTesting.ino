#include <TimeLib.h>

int validTimes[32] = {700, 730, 800, 830, 900, 930, 1000, 1015, 1030, 1045, 1100, 1115, 1130, 1145, 1200, 1215, 1230, 1245, 1300, 1315, 1330, 1345, 1400, 1415, 1430, 1445, 1500, 1515, 1530, 1600, 1630, 1700};
int sensorVals[32] = {00000, 00001, 00010, 00011, 00100, 00101, 00110, 00111, 01000, 01001, 01010, 01011, 01100, 01101, 01110, 01111, 10000, 10001, 10010, 10011, 10100, 10101, 10110, 10111, 11000, 11001, 11010, 11011, 11100, 11101, 11110, 11111};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int hours = hour();
  int minutes = minute();
  int seconds = second();
  int timeVal = (hours * 100) + minutes;
  int sensor = findTime(timeVal);
  if(sensor > -1){
    int sensorNumber = sensorVals[sensor];
  }
  Serial.println(timeVal);
}


int findTime(int timeVal){
  for(int i = 0; i < 32; i++){
    if(timeVal == validTimes[i]){
      return i; 
    }
  }
  return -1; 
}
