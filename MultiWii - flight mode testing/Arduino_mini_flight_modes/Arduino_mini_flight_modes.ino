#include <SoftwareSerial.h>      //needed for SoftwareSerial
SoftwareSerial mySerial(8, 7);   // RX, TX

#define MOTOR_ARM       10
#define MOTOR_DISARM	11

#define ANGLE_ON		20
#define ANGLE_OFF		21

#define HORIZON_ON		30
#define HORIZON_OFF		31

#define BARO_ON			40
#define BARO_OFF		41

#define MAG_ON			50
#define MAG_OFF			51

#define HEADFREE_ON		60
#define HEADFREE_OFF	61

#define HEADADJ_ON		70
#define HEADADJ_OFF		71

#define FAILSAFE_ON		80
#define FAILSAFE_OFF	81

unsigned int ARM_STATUS = MOTOR_DISARM;
unsigned int ANGLE_MODE = ANGLE_OFF;
unsigned int HORIZON_MODE = HORIZON_OFF;
unsigned int BARO_MODE = BARO_OFF;
unsigned int HEADFREE_MODE = HEADFREE_OFF;
unsigned int HEADADJ_MODE = HEADADJ_OFF;
unsigned int FAILSAFE_MODE = FAILSAFE_OFF;

long time;

void setup() {
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  Serial.begin(9600);
  
  // set the data rate for the SoftwareSerial port, max at 57600
  mySerial.begin(57600);
}

void loop(){
	time++;
	
	switch(time){
	case 50000: //Angle mode toggle
		ANGLE_MODE = ANGLE_ON;
		mySerial.write(ANGLE_MODE);
                Serial.println("Angle mode on");
	break;
	
	case 100000: //Horizon mode toggle
		ANGLE_MODE = ANGLE_OFF;
		HORIZON_MODE = HORIZON_ON;
		mySerial.write(HORIZON_MODE);
                Serial.println("Horizon mode on");
	break;
	
	case 150000: //Baro mode toggle
		HORIZON_MODE = HORIZON_OFF;
		BARO_MODE = BARO_ON;
		mySerial.write(BARO_MODE);
                Serial.println("Baro mode on");
	break;
	
	case 200000: //Head free mode toggle
		BARO_MODE = BARO_OFF;
		HEADFREE_MODE = HEADFREE_ON;
		mySerial.write(HEADFREE_MODE);
                Serial.println("Headfree mode on");
	break;
	
	}
	
	if(time > 250000){ 
	time = 0;
	HEADFREE_MODE = HEADFREE_OFF;
	}
}
