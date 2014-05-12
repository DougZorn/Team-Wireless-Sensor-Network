#include <SoftwareSerial.h>      //needed for SoftwareSerial
SoftwareSerial mySerial(8, 7);   // RX, TX
/*
#define MOTOR_ARM       11
#define MOTOR_DISARM	10

#define ANGLE_ON	21
#define ANGLE_OFF	20

#define HORIZON_ON	31
#define HORIZON_OFF	30

#define BARO_ON		41
#define BARO_OFF	40

#define MAG_ON		51
#define MAG_OFF		50

#define HEADFREE_ON	61
#define HEADFREE_OFF	60

#define HEADADJ_ON	71
#define HEADADJ_OFF	70

#define FAILSAFE_ON	81
#define FAILSAFE_OFF	80
*/

const byte MOTOR_ARM  = 0x0B;
const byte MOTOR_DISARM	= 0x0A;

const byte ANGLE_ON = 0x15;
const byte ANGLE_OFF = 	0x14;

const byte HORIZON_ON	 = 0x1F;
const byte HORIZON_OFF	 = 0x1E;

const byte BARO_ON     =  0x29;
const byte BARO_OFF	 = 0x28;

const byte MAG_ON       =  0x33;
const byte MAG_OFF	 = 0x32;

unsigned int ARM_STATUS = MOTOR_DISARM;
unsigned int ANGLE_MODE = ANGLE_OFF;
unsigned int HORIZON_MODE = HORIZON_OFF;
unsigned int BARO_MODE = BARO_OFF;
//unsigned int HEADFREE_MODE = HEADFREE_OFF;
//unsigned int HEADADJ_MODE = HEADADJ_OFF;
//unsigned int FAILSAFE_MODE = FAILSAFE_OFF;

int temp = 0;
boolean modeChanged = false;
boolean toggleValueSent = false;
unsigned int currTime = 0;
unsigned int lastTime = 0;
const unsigned int TIMEOUT = 800;


void setup() {
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port, max at 57600
  mySerial.begin(9600);

}

void loop(){
  while(!toggleFlightModes(HORIZON_ON)){
  }
  
  delay(3000);
  
  while(!toggleFlightModes(HORIZON_OFF)){
  }
  
  delay(3000);
}


boolean toggleFlightModes(byte modeValue){
  boolean modeChanged = false;
  temp = 0;
  if(mySerial.available() > 0){  // this checks the rx buffer if there is anything there, Buffer_Size = 64Byte
    temp = mySerial.read();  //read one byte of data from buffer to temp
    Serial.print("data from CB: ");
    Serial.println(temp);
  }
  if(temp == modeValue){
    Serial.print("Mode toggled: ");
    Serial.println(modeValue);
    modeChanged = true;
    toggleValueSent = false;
    lastTime = 0;
    currTime = 0;
    return modeChanged;
  }
  else if (!toggleValueSent){
    mySerial.write(modeValue);
    Serial.print("sent to CB: ");
    Serial.println(modeValue);
    toggleValueSent = true;
    lastTime = millis();
    currTime = 0;
  }
  currTime = millis() - lastTime;
  if (currTime > TIMEOUT) toggleValueSent = false;
  
  return modeChanged;
}



