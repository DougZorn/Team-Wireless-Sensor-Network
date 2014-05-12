
#include "Arduino.h"
#include "Alarms.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"
#include "Serial.h"


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

#define FAILSAFE_ON	81
#define FAILSAFE_OFF	80


unsigned int lastOrder = 0;
unsigned int newOrder = 1;

void checkNode(){
  if(SerialAvailable(1)>0){    //Bytes in the RX buffer
    newOrder = SerialRead(1);
    //if(newOrder != lastOrder){
    switch(newOrder){
      case (HORIZON_ON):
      rcData[AUX1] = 2000;       //Set spoofed RX value for horizon channel
      SerialWrite(1, newOrder);  //Send back horizon_on value to confirm order received
      rcOptions[BOXHORIZON] = 1; //Toggle change in flight mode
      break;

      case (HORIZON_OFF):
      rcData[AUX1] = 1000;       //Set spoofed RX value for horizon channel
      SerialWrite(1, newOrder);  //Send back horizon_off value to confirm order received
      rcOptions[BOXHORIZON] = 0; //Toggle change in flight mode
      break;

      case (BARO_ON):
      rcData[AUX2] = 2000;      //Set spoofed RX value for baro channel
      rcOptions[BOXBARO] = 1;   //Send back baro_on value to confirm order received
      SerialWrite(1, newOrder); //Toggle change in flight mode
      break;

      case (BARO_OFF):
      rcData[AUX2] = 1000;      //Set spoofed RX value for baro channel
      rcOptions[BOXBARO] = 0;   //Send back baro_off value to confirm order received
      SerialWrite(1, newOrder); //Toggle change in flight mode
      break;

      case (MAG_ON):
      rcData[AUX3] = 2000;      //Set spoofed RX value for mag channel
      rcOptions[BOXMAG] = 1;    //Send back mag_on value to confirm order received
      SerialWrite(1, newOrder); //Toggle change in flight mode
      break;

      case (MAG_OFF):
      rcData[AUX3] = 1000;      //Set spoofed RX value for mag channel
      rcOptions[BOXMAG] = 0;    //Send back mag_off value to confirm order received
      SerialWrite(1, newOrder); //Toggle change in flight mode
      break;

      /*
      case (ANGLE_ON):
       rcData[AUX4] = 1000;       //Set spoofed RX value for baro channel
       rcOptions[BOXANGLE] = 0;   //Send back ANGLE_ON value to confirm order received
       SerialWrite(1, newOrder);  //Toggle change in flight mode
       break;

       case (ANGLE_OFF):
       rcData[AUX4] = 1000;       //Set spoofed RX value for baro channel
       rcOptions[BOXANGLE] = 0;   //Send back ANGLE_OFF value to confirm order received
       SerialWrite(1, newOrder);  //Toggle change in flight mode
       break;

       case (HEADFREE_ON):
       rcData[AUX4] = 1000;          //Set spoofed RX value for mag channel
       rcOptions[BOXHEADFREE] = 0;   //Send back HEADFREE_ON value to confirm order received
       SerialWrite(1, newOrder);     //Toggle change in flight mode
       break;

       case (HEADFREE_OFF):
       rcData[AUX4] = 1000;         //Set spoofed RX value for baro channel
       rcOptions[BOXHEADFREE] = 0;  //Send back HEADFREE_ON value to confirm order received
       SerialWrite(1, newOrder);    //Toggle change in flight mode
       break;
       */
    }
    lastOrder = newOrder;
    //}
  }
}
void maintainNode(){

  if(f.HORIZON_MODE)
    rcData[AUX1] = 2000;

  if(f.HORIZON_MODE  == 0)
    rcData[AUX1] = 1000;

  if(f.BARO_MODE)
    rcData[AUX2] = 2000;

  if(f.BARO_MODE == 0)
    rcData[AUX2] = 1000;

  if(f.MAG_MODE)
    rcData[AUX3] = 2000;

  if(f.MAG_MODE  == 0)
    rcData[AUX3] = 1000;
}





