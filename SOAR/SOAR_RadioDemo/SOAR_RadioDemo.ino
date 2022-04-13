/*-------------------------------------------------------------
  SOAR radio demo code
  Devin Spivey and Fowler Askew
  AURA Payload 2021-22
  -------------------------------------------------------------*/
#define LED_2 22
#define LED_1 23

#include <RadioLib.h>  // include radio library

// motor pins
#define PWM_A 6  // PWM pin, motor A (SOAR)
#define DIR_A 5  // Direction pin, motor A
#define PWM_B 8  // PWM pin, motor B (SLS)
#define DIR_B 7  // Direction pin, motor B

#define LEGS_DIR_1 1
#define LEGS_PWM_1 2
#define LEGS_DIR_2 3
#define LEGS_PWM_2 4

// RFM97 connections:
#define CSPIN 10
#define DIO0PIN 14
#define NRSTPIN 9 
#define DIO1PIN 15

RFM97 radio = new Module(CSPIN, DIO0PIN, NRSTPIN, DIO1PIN);  // radio object
bool isLaunched = false;  // flag for when launch has occurred
bool isLanded = false;

// transmit variables
unsigned int transmitTimer = 0;  // stores the time of the last transmission
unsigned int transmitInterval = 2500;  // milliseconds between tranmissions

// receive array
byte RXarray[8];  // stores received array

// radio variables
int transmitState = RADIOLIB_ERR_NONE;  // saves radio state when transmitting
int receiveState = RADIOLIB_ERR_NONE;  // saves radio state when receiving
bool enableInterrupt = true;  // disables interrupt when not needed
volatile bool operationDone = false;  // indicates an operation is complete
bool wasTX = false;  // indicates the last operation was transmission
bool txComplete = true;  // indicates the last transmission is done
float lastRSSI = 0;  // saves RSSI to be transmitted

// control variables
int armVar = 0;
int soarVar = 0;
int slsVar = 0;
int legsVar = 0;

void setup()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(LEGS_PWM_1, OUTPUT);
  pinMode(LEGS_DIR_1, OUTPUT);
  pinMode(LEGS_PWM_2, OUTPUT);
  pinMode(LEGS_DIR_2, OUTPUT);

  digitalWrite(LED_2, HIGH);
  digitalWrite(PWM_A, LOW);  // Set all these things low so that nothing happens
  digitalWrite(DIR_A, LOW);
  digitalWrite(PWM_B, LOW);
  digitalWrite(DIR_B, LOW);
  digitalWrite(LEGS_PWM_1, LOW);
  digitalWrite(LEGS_DIR_1, LOW);
  digitalWrite(LEGS_PWM_2, LOW);
  digitalWrite(LEGS_DIR_2, LOW);

  Serial.begin(115200);
  delay(250);

  // ----- BEGIN RADIO SETUP -----
  // initialize RFM97 with all settings listed
  Serial.print(F("[RFM97] Initializing ... "));
  receiveState = radio.begin(915.0,  // carrier freq (MHz)
                             125.0,  // bandwidth (kHz)
                             9,  // spreading factor
                             7,  // coding rate
                             0x12,  // sync word
                             17,  // output power (dBm)
                             8,  // preamble length (symbols)
                             0);  // gain (0 is automatic control)
  radio.setCRC(true);  // enables cyclic redundancy check
  delay(500);

  if (receiveState == RADIOLIB_ERR_NONE)  // if radio initialized correctly
  {
    Serial.println(F("init success!"));
    for(int l=0; l < 3; l++)
    {
      digitalWrite(LED_1, HIGH);
      delay(100);
      digitalWrite(LED_1, LOW);
      delay(100);
    }
  }
  else
  {
    Serial.print(F("failed, code "));  // print error code
    Serial.println(receiveState);
  }

  radio.setDio0Action(setFlag);  // function that will be called when something is done
  // ----- END RADIO SETUP -----

  setMotors();  // sets the motors based on controls array
  
  Serial.println("Startup complete");
  delay(1000);
  
  Serial.println("Listening for packets");
  receiveState = radio.startReceive();  // start listening
  digitalWrite(LED_2, LOW);
}

void loop()
{
  
  checkRadio();
}

void handleReceive()  // performs everything necessary when data comes in
{
  receiveState = radio.readData(RXarray, 8);  // read received data to array

  if (receiveState == RADIOLIB_ERR_NONE)  // packet received correctly
  {
    Serial.println(F("[RFM97] Received packet!"));

    Serial.print(F("[RFM97] Data:\t\t"));  // print data
    Serial.print(RXarray[0]);
    Serial.print("\t");
    Serial.print(RXarray[1], BIN);
    Serial.print("\t");
    Serial.print(RXarray[2]);
    Serial.print("\t");
    Serial.print(RXarray[3]);
    Serial.print("\t");
    Serial.print(RXarray[4]);
    Serial.print("\t");
    Serial.print(RXarray[5]);
    Serial.print("\t");
    Serial.print(RXarray[6]);
    Serial.print("\t");
    Serial.print(RXarray[7]);
    Serial.print("\t");
    Serial.println(RXarray[8]);
    
    Serial.print(F("\t[RFM97] RSSI: "));  // print RSSI if desired
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    if(RXarray[0] == 0)  // if the values are from MARCO, update the stuff
    {
      armVar = RXarray[1] & 0b00000001;  // armVar is set to the state of the arm bit
      
      soarVar = RXarray[3];  // SOAR motor speed from RXarray
      if(~RXarray[1] & 0b00000100)  // if SOAR direction bit is not set
        soarVar *= -1;
      
      slsVar = RXarray[4];  // SLS speed from RXarray
      if(~RXarray[1] & 0b00001000)  // if SLS direction bit is not set
        slsVar *= -1;
      
      legsVar = RXarray[5];  // LEGS speed from RXarray
      if(~RXarray[1] & 0b00010000)  // if LEGS direction bit is not set
        legsVar *= -1;
    }
  }
}

void checkRadio()  // this checks if a radio operation is complete and restarts radio operation
{
  if(operationDone)  // if the last radio operation is finished
  {
    digitalWrite(LED_1, LOW);  // No LED in between modes
    enableInterrupt = false;  // disable the interrupt
    operationDone = false;  // reset completion flag

    if(wasTX)  // last action was transmit
    {
      wasTX = false;  // not transmitting this time
      txComplete = true;
      receiveState = radio.startReceive();  // start receiving again
      digitalWrite(LED_1, HIGH);  // LED 1 on while receive mode is active
    }

    else  // last action was receive
    {
      handleReceive();  // this function does stuff based on received commands
      setMotors();  // sets the motors based on controls array - not sure if this is the best place to do this
      delay(2);  // delay a little to give some time for other systems
      transmitData();  // Retransmit data - it would likely be better to break this out into the main loop
    }
    enableInterrupt = true;  // reenable the interrupt
  }
}

void transmitData()  // this function just retransmits the received array with a new system address
{
  RXarray[0] = 2;  // set SOAR's system address
  if(isLanded)
    RXarray[7] = 2;
  else if(isLaunched)
    RXarray[7] = 1;
  else
    RXarray[7] = 0;
  
  wasTX = true;
  txComplete = false;
  transmitTimer = millis();  // reset transmit timer
  
//  Serial.println(F("[RFM97] Sending array ... "));
  transmitState = radio.startTransmit(RXarray, 8);  // transmit array
}

void setFlag(void)  // this function is called after complete packet transmission or reception
{
  if(!enableInterrupt)  // check if the interrupt is enabled
    return;

  operationDone = true;  // something happened, set the flag
}

void setMotors()  // This will print values for motor speeds
{
  Serial.print("SOAR motor: ");
  Serial.print(soarVar);
  Serial.print("\tSLS motor: ");
  Serial.print(slsVar);
  Serial.print("\tLEGS motor1: ");
  Serial.print(legsVar);
  Serial.print("\tLEGS motor2: ");
  Serial.println(legsVar);
}
