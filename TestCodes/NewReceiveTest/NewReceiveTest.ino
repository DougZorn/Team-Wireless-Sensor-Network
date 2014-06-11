/*  
 *  
 *  Slave Node Code
 *  
 *  Before using, check:
 *  NUM_NODES, MY_NAME, PREV_NODE, PREV_PREV_NODE
 *
 */
#include <SPI.h>
#include <string.h>
#include <SoftwareSerial.h>      //needed for SoftwareSerial
#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
//#include "read_write.h"
#include "read_writeV2.h"
#include "motorcontrol.h"

int roundcount = 0;
char Packet[11]= {0x0A,0xFF,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
byte recvPacket[9]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Check = 0;

int updateData(byte *array);

void setup()
{ 
  Serial.begin(9600); 
  init_CC2500_V2();  
}

void loop()
{ 
  /*
  if(roundcount++ >50){  
   sendPacket2(Packet,0x0B);
   roundcount =0;
  }*/
  
  if(1==listenForPacket2(recvPacket,0x09)){
    for(int x =0; x<9;x++){
      Serial.println(recvPacket[x],HEX); 
    }
    Serial.println("");
  }
} 

int updateData(byte *array){  //Ultra Sonic will still update even if uart does not update data when calling this

  myData.ultraSonic = analogRead(UltraSonicPin);    // update ultraSonic data no matter what, it does not use uart from control board

  if(upDated !=1){          //see if update from uart
    return 1;               
  }
  byte sensorType;
  byte startByte = 0x80;    //Byte indicating the Start of chain of packets
  byte endByte  = 0xC0;     //Byte indicating the End of chain of packets
  int16_t tempData16;                   
  int32_t tempData32;
  
  //int flag = 0;
  int place = 0;            //Locate where packet in array starts, eliminates garbage in front of start if any
  
  while((array[place] != startByte)&&(place <64)){    //locate where start of packet is
    place++;
    //Serial.print("here: ");
    //Serial.println(place);
  } 
  
  if(place>=64){      //If not start of packet is found, return fail
    return 1;
  }
  place++;            //move to next byte after start packet
  
  do{
    sensorType = array[place];
    if(sensorType < 32){        //look at the type of data in the packet, there are usually multiple different types, all type < 32 are or int16 sensors
      place++;                                   // move there
      tempData16 = array[place];                //assemble them because they are int16 or int 32 and uart only sents bytes
      
      //Serial.print(" Data_x1: ");
      //Serial.print(tempData16, HEX);
      tempData16 = tempData16 <<8;
      //Serial.print(" Data_x2: ");
      //Serial.print(tempData32, HEX);
      
      place++;
      tempData16 += array[place];;
      //Serial.print(" Type: ");
      //Serial.print(sensorType, HEX);
      //Serial.print(" Data: ");
      //Serial.println(tempData16, HEX);
      storeData16(sensorType, tempData16);    //store the assembled data into cor
    }else if(sensorType <64){                                    //if int32 sensors
      place++;                               //assemble
      tempData32 = array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      //Serial.print("Type: ");
      //Serial.print(sensorType, HEX);
      //Serial.print(" Data: ");
      //Serial.println(tempData32, HEX);
      storeData32(sensorType, tempData32);    //store it in correct place
    }else{
      place++;                                   // move there
      currACK = array[place];          
      if(currACK != prevACK){
        changeMode(currACK);
        prevACK = currACK;
      }
    }
     
    place++;
    if(place>=64){    //if it reach the max buffer size and array size without seeing End Byte, it is cuted off
      revertData();  //put old data back and return fail
      return 1;  
    }
  }while(array[place]!=endByte);  //loop until seeing endByte
  
  for(int CT; CT<64;CT++){      //reset the array for storing data
    array[CT] = 0;
  }
  mySerial.flush();            //flush everything left in the uart rx buffer, size 64 bytes
  
  return 0;
  
}

