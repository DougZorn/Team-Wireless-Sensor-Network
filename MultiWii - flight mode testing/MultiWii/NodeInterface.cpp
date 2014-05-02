#ifndef NODEINTERFACE_H_
#define NODEINTERFACE_H_

#define MOTOR_ARM       10
#define MOTOR_DISARM	11

#define ANGLE_ON	20
#define ANGLE_OFF	21

#define HORIZON_ON	30
#define HORIZON_OFF	31

#define BARO_ON		40
#define BARO_OFF	41

#define MAG_ON		50
#define MAG_OFF		51

#define HEADFREE_ON	60
#define HEADFREE_OFF	61

#define HEADADJ_ON	70
#define HEADADJ_OFF	71

#define FAILSAFE_ON	80
//#define FAILSAFE_OFF	81

#define READ_SENSORS 1

const uint32 TIMEOUT 300;

uint8 lastOrder;
uint8 newOrder;
unit8 modeStatus;
uint32 currentTime;
uint32 previousTime;

void Node_init() {
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  // set the data rate for the SoftwareSerial port, max at 57600
  // Make sure it is in sync with the Arduino baud rate
  SerialOpen(1,57600);
}


void checkNode{}{
//check serial, if new serial, & different from last time, or same sensor ask from node and timeout has occured
//interpret value
//if input == 1, send package of sensor data 
//start timer for sensor ask timeout

//if greater than 10, new flight mode has been applied
//check for interference with flight modes
//assign flight modes

//If there is a byte in the TX buffer
  if(SerialAvailable(1)>0){						
	currentTime = millis() - lastTime;
	newOrder = SerialRead(1);
	if(newOrder != lastOrder){ 				//Order is new, now interpret
		modeStatus = newOrder % 10;
		switch(newOrder){
		case (MOTOR_ARM || MOTOR_DISARM):
			if(f.OK_TO_ARM && !f.ARMED && modeStatus == 1)
				f.ARMED = modeStatus;
			if(f.ARMED && modeStatus == 0)
				f.ARMED = modeStatus;
		break;
		
		case (ANGLE_ON || ANGLE_OFF):
			if( !f.ANGLE_MODE && modeStatus == 1){
				f.ANGLE_MODE = modeStatus;
				if(HORIZON_MODE) HORIZON_MODE = 0; 		//CANNOT HAVE ANGLE AND HORIZON ON AT THE SAME TIME
				}
			if( f.ANGLE_MODE && modeStatus == 0)
				f.ANGLE_MODE = modeStatus;
		break;
		
		case (HORIZON_ON || HORIZON_OFF):
			if( !f.HORIZON_MODE && modeStatus == 1){
				f.HORIZON_MODE = modeStatus;
				if(ANGLE_MODE) HORIZON_MODE = 0; 		//CANNOT HAVE ANGLE AND HORIZON ON AT THE SAME TIME
				}
			if( f.HORIZON_MODE && modeStatus == 0)
				f.HORIZON_MODE = modeStatus;
		break;
		
		case (BARO_ON || BARO_OFF):
			if(!f.BARO_MODE && modeStatus == 1){
				f.BARO_MODE = modeStatus;
				}
			if(f.BARO_MODE && modeStatus == 0)
				f.BARO_MODE = modeStatus;
		break;
		
		case (MAG_ON || MAG_OFF):
			if(!f.MAG_MODE && modeStatus == 1){
				f.MAG_MODE = modeStatus;
				}
			if(f.MAG_MODE && modeStatus == 0)
				f.MAG_MODE = modeStatus;
		break;
		
		case (HEADFREE_ON || HEADFREE_OFF):
			if(!f.HEADFREE_MODE && modeStatus == 1){
				f.HEADFREE_MODE = modeStatus;
				}
			if(f.HEADFREE_MODE && modeStatus == 0){
				f.HEADFREE_MODE = modeStatus;				//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_FREE is mode is disabled, so is HEAD_ADJ
				f.HEADADJ_MODE = modeStatus;
				}
		break;
		
		case (HEADADJ_ON || HEADADJ_OFF):
			if(!f.HEADADJ_MODE && modeStatus == 1){
				f.HEADADJ_MODE = modeStatus;
				}
			if(f.HEADADJ_MODE && modeStatus == 0){
				f.HEADADJ_MODE = modeStatus;				//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_ADJ is mode is disabled, so is HEAD_FREE
				f.HEADFREE_MODE = modeStatus;
				}
		break;
		
		case (FAILSAFE_ON):
			//STUB
		break;
		
		case (READ_SENSORS)::
			//STUB
		break;
		
		//default:
		//break;
		}
		lastTime = millis(); //Reset time since last new order
		lastOrder = newOrder;
		} else {
		
			if(newOrder == 1 && currentTime > TIMEOUT){		//Order is repeat sensor ask from node
				//Send sensor data package
				lastTime = millis(); //Reset time since last new order
			}
		}
	}
}


/*
Data Structure of Sensor Data and other stuff

typedef struct {                    array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            smoother version of accADC
  int16_t  gyroData[3];             Not sure
  int16_t  magADC[3];              180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             raw gyro data
  int16_t  accADC[3];              raw accelerometer data
} imu_t;

typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;
} analog_t;

typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt_t;

typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;
*/

//Note:  Do Not Spam the RX buffere or TX buffer on the Mini or the Control Board
//       it will generate error. 

// SerialAvailable(port number) The port we use is 1. this return the noumber of avaible bytes in the RX buffer
//  EX. uint8_t x = SerialAvailable(1); , if(SerialAvailable(1)>0)

// SerialRead(port number) returns one byte of data from the RX Buffer, uint8_t
// EX. uint8_t y = SerialRead(1);

//SerialUsedTXBuff(port number) returns the number of bytes stored in TX buffer, leave 50byte of space to avoid error

//SerialWrite(port number, one byte of Data) Writes one byte of data to TX buffer, flag TX ISR, Some time later, TX ISR will transmitt all in buffer
