
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"
#include "Serial.h"


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

#define READ_SENSORS 1

const unsigned int TIMEOUT = 300;

unsigned int lastOrder;
unsigned int newOrder;
unsigned int modeStatus;
unsigned int currTime;
unsigned int prevTime;

void checkNode(){
//check serial, if new serial, & different from last time, or same sensor ask from node and timeout has occured
//interpret value
//if input == 1, send package of sensor data 
//start timer for sensor ask timeout

//if greater than 10, new flight mode has been applied
//check for interference with flight modes
//assign flight modes

//If there is a byte in the TX buffer
  if(SerialAvailable(1)>0){						
	currTime = millis() - prevTime;
	newOrder = SerialRead(1);
	if(newOrder != lastOrder){ 				//Order is new, now interpret
             SerialWrite(1, newOrder);
             //SerialWrite(1, f.ANGLE_MODE);
             modeStatus = newOrder % 10;
		switch(newOrder){
    		case (MOTOR_ARM):
    			if(f.OK_TO_ARM && !f.ARMED && modeStatus == 1)
    				f.ARMED = modeStatus;
    		break;
    
                    case (MOTOR_DISARM):
    			if(f.ARMED && modeStatus == 0)
    				f.ARMED = modeStatus;
    		break;
    		
    		case (ANGLE_ON):
    			if( !f.ANGLE_MODE && modeStatus == 1){
    				f.ANGLE_MODE = modeStatus;
                                SerialWrite(1, f.ANGLE_MODE);
    				if(f.HORIZON_MODE) f.HORIZON_MODE = 0; 		//CANNOT HAVE ANGLE AND HORIZON ON AT THE SAME TIME
    				}
    		break;
    
                    case (ANGLE_OFF):
    			if( f.ANGLE_MODE && modeStatus == 0){
    				f.ANGLE_MODE = modeStatus;
                                SerialWrite(1, f.ANGLE_MODE);
                         }
    		break;
    		
    		case (HORIZON_ON):
    			if( !f.HORIZON_MODE && modeStatus == 1){
    				f.HORIZON_MODE = modeStatus;
    				if(f.ANGLE_MODE) f.ANGLE_MODE = 0; 		//CANNOT HAVE ANGLE AND HORIZON ON AT THE SAME TIME
    				}
    		break;
    
    		case (HORIZON_OFF):
    			if( f.HORIZON_MODE && modeStatus == 0)
    				f.HORIZON_MODE = modeStatus;
    		break;
    		
    		case (BARO_ON):
    			if(!f.BARO_MODE && modeStatus == 1){
    				f.BARO_MODE = modeStatus;
    				}
    		break;
    		
    		case (BARO_OFF):
    			if(f.BARO_MODE && modeStatus == 0)
    				f.BARO_MODE = modeStatus;
    		break;
    
    		case (MAG_ON):
    			if(!f.MAG_MODE && modeStatus == 1){
    				f.MAG_MODE = modeStatus;
    				}
    		break;
    
    		case (MAG_OFF):
    			if(f.MAG_MODE && modeStatus == 0)
    				f.MAG_MODE = modeStatus;
    		break;
    		
    		case (HEADFREE_ON):
    			if(!f.HEADFREE_MODE && modeStatus == 1){
    				f.HEADFREE_MODE = modeStatus;
    				}
    		break;
    
    		case (HEADFREE_OFF):
    			if(f.HEADFREE_MODE && modeStatus == 0){
    				f.HEADFREE_MODE = modeStatus;				//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_FREE is mode is disabled, so is HEAD_ADJ
    				//f.HEADADJ_MODE = modeStatus;
    				}
    		break;
    		
    		/*case (HEADADJ_ON || HEADADJ_OFF):
    			if(!f.HEADADJ_MODE && modeStatus == 1){
    				f.HEADADJ_MODE = modeStatus;
    				}
    			if(f.HEADADJ_MODE && modeStatus == 0){
    				f.HEADADJ_MODE = modeStatus;				//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_ADJ is mode is disabled, so is HEAD_FREE
    				f.HEADFREE_MODE = modeStatus;
    				}
    		break; */
    		
    		case (FAILSAFE_ON):
    			//STUB
    		break;
    		
    		case (READ_SENSORS):
    			//STUB
    		break;
    		
    		//default:
    		//break;
		}
		prevTime = millis(); //Reset time since last new order
		lastOrder = newOrder;
		} else {
		
			if(newOrder == 1 && currentTime > TIMEOUT){		//Order is repeat sensor ask from node
				//Send sensor data package
				prevTime = millis(); //Reset time since last new order
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
