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

long time =0;
int temp;

void setup() {
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port, max at 57600
  mySerial.begin(9600);

}

void loop(){
  if(mySerial.available() > 0){  // this checks the rx buffer if there is anything there, Buffer_Size = 64Byte
    temp = mySerial.read();  //read one byte of data from buffer to temp
    Serial.print("data: ");
    Serial.println(temp);
  }
  //mySerial.write(ANGLE_MODE);
  
  if(time < 125000 || (time > 125000 && time < 250000) || time > 250000)  time++;
  
  switch(time){
  case 125000: //Angle mode toggle
  HORIZON_MODE = HORIZON_ON;
  //for(int i = 0; i < 10;i++)
    mySerial.write(HORIZON_MODE);
    if(temp == HORIZON_MODE){
      time++;
      Serial.println("Horizon mode on");
    }
  Serial.print("sent: ");
  Serial.println(HORIZON_MODE);
  
  break;

  
  case 250000: //angle mode toggle
  HORIZON_MODE = HORIZON_OFF;
  //for(int i = 0; i < 10;i++)
    mySerial.write(HORIZON_MODE);
    if(temp == HORIZON_MODE){
      time++;
      Serial.println("Horizon mode off");
    }
  //mySerial.write(ANGLE_MODE);
  Serial.print("sent: ");
  Serial.println(HORIZON_MODE);
  break; /*
 
   case 100000: //Horizon mode toggle
   		ANGLE_MODE = ANGLE_OFF;
   		HORIZON_MODE = HORIZON_ON;
   		mySerial.write(HORIZON_MODE);
   		//mySerial.write(HORIZON_MODE);
   		//mySerial.write(HORIZON_MODE);
   Serial.println("Horizon mode on");
   	break;

   	case 150000: //Baro mode toggle
   		HORIZON_MODE = HORIZON_OFF;
   		BARO_MODE = BARO_ON;
   		mySerial.write(BARO_MODE);
   		//mySerial.write(BARO_MODE);
   		//mySerial.write(BARO_MODE);
   Serial.println("Baro mode on");
   	break;

   	case 200000: //Head free mode toggle
   		BARO_MODE = BARO_OFF;
   		HEADFREE_MODE = HEADFREE_ON;
   		mySerial.write(HEADFREE_MODE);
   //mySerial.write(HEADFREE_MODE);
   //mySerial.write(HEADFREE_MODE);
   Serial.println("Headfree mode on");
   	break;
   	*/
  }

  if(time > 250000){
    time = 0;
  }
}


