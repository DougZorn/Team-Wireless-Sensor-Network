
#include "Arduino.h"
#include "Alarms.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"
#include "Serial.h"
#include "Nodeinterface.h"


#define MOTOR_ARM 0x0B
#define MOTOR_DISARM 0x0A

#define ANGLE_ON 0x15
#define ANGLE_OFF 0x14

#define HORIZON_ON 0x1F
#define HORIZON_OFF 0x1E

#define BARO_ON 0x29
#define BARO_OFF 0x28

#define MAG_ON 0x33
#define MAG_OFF 0x32

#define SENSOR_UPDATE 0x5A

int16_t sensorDataU = 200;


byte lastOrder = 0;
byte newOrder = 1;
byte assemble_temp1 = 0;
byte assemble_temp2 = 0;

byte checkSum;

byte checkNode()
{  
  if(SerialAvailable(1)>0)
  {    //Bytes in the RX buffer
    newOrder = SerialRead(1);
    switch(newOrder)
    {
      case (HORIZON_ON):      
      rcOptions[BOXHORIZON] = 1; //Toggle change in flight mode
      return newOrder;

      case (HORIZON_OFF):      
      rcOptions[BOXHORIZON] = 0; //Toggle change in flight mode
      return newOrder;

      case (BARO_ON):      
      rcOptions[BOXBARO] = 1;   //Send back baro_on value to confirm order received
      return newOrder;     

      case (BARO_OFF):
      rcOptions[BOXBARO] = 0;   //Send back baro_off value to confirm order received
      return newOrder;

      case (MAG_ON):      
      rcOptions[BOXMAG] = 1;    //Send back mag_on value to confirm order received
      return newOrder;      

      case (MAG_OFF):
      rcOptions[BOXMAG] = 0;    //Send back mag_off value to confirm order received
      return newOrder;
      
      case (SENSOR_UPDATE):
        assemble_temp1 = 0;
        assemble_temp2 = 0;
        checkSum = 0;
        assemble_temp1 = SerialRead(1);
        assemble_temp2 = SerialRead(1);
        checkSum = SerialRead(1);
        
        if(((checkSum == ((assemble_temp1 + assemble_temp2)%10)))&&((assemble_temp1 != 0)||(assemble_temp2 != 0))){
          sensorDataU = assemble_temp1 <<8;
          sensorDataU += assemble_temp2;
        }
        return 0x00;     
      }
  }
  return 0x00;
}





