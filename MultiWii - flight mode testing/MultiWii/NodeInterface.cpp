
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

unsigned int lastOrder = 0;
unsigned int newOrder = 1;
unsigned int modeStatus;
unsigned int currTime;
unsigned int prevTime;

void setMode(){
  int t = 0;
  t++;

  if(!rcOptions[BOXHORIZON])
    rcOptions[BOXHORIZON] = 1;
  if(t>50000){
    t = 0;
    rcOptions[BOXHORIZON] = 0;
  }
}

void checkNode(){
  if(SerialAvailable(1)>0){
    newOrder = SerialRead(1);
    //SerialWrite(1, newOrder);
    //if(newOrder != lastOrder){ 				//Order is new, now interpret
      //modeStatus = newOrder % 10;
      switch(newOrder){
        case (MOTOR_ARM):
        if(f.OK_TO_ARM && !f.ARMED)
          rcOptions[BOXARM] = 1;
        break;

        case (MOTOR_DISARM):
        if(f.ARMED)
          rcOptions[BOXARM] = 0;
        break;

        case (ANGLE_ON):
        rcData[AUX4] = 2000;
        if( !f.ANGLE_MODE){
          rcOptions[BOXANGLE] = 1;
          SerialWrite(1, newOrder);
        }
        break;

        case (ANGLE_OFF):
        rcData[AUX1] = 1000;
        if( f.ANGLE_MODE){
          rcOptions[BOXANGLE] = 0;
          SerialWrite(1, newOrder);
        }
        break;

        case (HORIZON_ON):
        rcData[AUX1] = 2000;
        SerialWrite(1, newOrder);
        rcOptions[BOXHORIZON] = 1;
        break;

        case (HORIZON_OFF):
        rcData[AUX1] = 1000;
        SerialWrite(1, newOrder);
        rcOptions[BOXHORIZON] = 0;
        break;

        case (BARO_ON):
        rcData[AUX2] = 2000;
        if(!f.BARO_MODE){
          rcOptions[BOXBARO] = 1;
          SerialWrite(1, newOrder);
        }
        break;

        case (BARO_OFF):
        rcData[AUX2] = 1200;
        if(f.BARO_MODE){
          rcOptions[BOXBARO] = 0;
          SerialWrite(1, newOrder);
        }
        break;

        case (MAG_ON):
        rcData[AUX3] = 2000;
        if(!f.MAG_MODE){
          rcOptions[BOXMAG] = 1;
          SerialWrite(1, newOrder);
        }
        break;

        case (MAG_OFF):
        rcData[AUX3] = 1000;
        if(f.MAG_MODE){
          rcOptions[BOXMAG] = 0;
          SerialWrite(1, newOrder);
        }
        break;

        /*
        case (HEADFREE_ON):
         if(!f.HEADFREE_MODE){
         rcOptions[BOXHEADFREE] = modeStatus;
         }
         break;

         case (HEADFREE_OFF):
         if(f.HEADFREE_MODE){
         rcOptions[BOXHEADFREE] = modeStatus;			//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_FREE is mode is disabled, so is HEAD_ADJ
         }
         break;

         case (HEADADJ_ON):
         if(f.HEADFREE_MODE){
         rcOptions[BOXHEADADJ] = modeStatus;
         }
         break;

         case (HEADADJ_OFF):
         if(f.HEADFREE_MODE){
         rcOptions[BOXHEADADJ] = modeStatus;				//HEAD_ADJ resets HEAD_FREE direction, so if HEAD_ADJ is mode is disabled, so is HEAD_FREE
         rcOptions[BOXHEADFREE] = modeStatus;
         }
         break;
         */

        //default:
        //break;
      }
      //lastOrder = newOrder;
    //}
  }
}
void maintainNode(){

  //if(rcOptions[BOXHORIZON]){ //f.HORIZON_MODE)
    //rcData[AUX1] = 2000;
    //f.HORIZON_MODE = 1;
    //rcOptions[BOXHORIZON] = 1;
  //}
   if(f.HORIZON_MODE){ //f.HORIZON_MODE)
    rcData[AUX1] = 2000;
    //f.HORIZON_MODE = 1;
    //rcOptions[BOXHORIZON] = 1;
  }
  //if((rcOptions[BOXHORIZON] == 0)){ //f.HORIZON_MODE  == 0) || 
    //rcData[AUX1] = 1000;
    //f.HORIZON_MODE = 0;
  //}
  
  if(f.HORIZON_MODE  == 0){
     rcData[AUX1] = 1000;
     //rcOptions[BOXHORIZON] = 0;
  } 

  //if(f.BARO_MODE || rcOptions[BOXBARO]) rcData[AUX2] = 2000;
  // else if(!f.BARO_MODE) rcData[AUX2] = 1000;

  //if(f.MAG_MODE || rcOptions[BOXMAG]) rcData[AUX3] = 2000;
  //else if(!f.MAG_MODE) rcData[AUX3] = 1000;

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



