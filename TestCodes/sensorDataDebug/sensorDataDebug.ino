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
char Packet[11]= {0x0A,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte recvPacket[9]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte Check = 0;

byte uartArray[64];

typedef struct {                    //array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            //smoother version of accADC
  int16_t  gyroData[3];             //Not sure
  int16_t  magADC[3];              //180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             //raw gyro data
  int16_t  accADC[3];              //raw accelerometer data
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
  int16_t  heading;
  uint32_t ultraSonic;        //in cm
} data_t;

data_t myData;

data_t recvData;

//Declare Pins for UART
SoftwareSerial mySerial(8, 7);   // RX, TX

//UltraSonic Stuff
int UltraSonicPin = A0;    // select the input pin for the potentiometer
uint32_t ultraSonic;        //in cm

int storeData32(int dType, int32_t data32);
int storeData16(int dType, int16_t data16);
int updateData(byte *array);

int upDated;
  
  
int curSpot =0;
  
void setup()
{ 
  Serial.begin(9600); 
  init_CC2500_V2();  
  
  mySerial.begin(9600);
  Serial.println("After SPI Init");
  
  Serial.println(ReadReg(REG_IOCFG0),HEX);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  Serial.println(ReadReg(REG_IOCFG2),HEX);
}

void loop()
{ /*
  upDated = 0;
  curSpot = 0;
  
  
    //Serial.println("newloop");
  
  while(mySerial.available()){ //maybe add || certain byte: hardcoded.
    //delayMicroseconds(5);    //millis here to avoid missed chained of bytes, dynamic code too restrictive 
    uartArray[curSpot++] = mySerial.read();
    upDated=1;
    if(curSpot>=64){
      mySerial.flush();
      break; 
    }
  }*/
  
  //if(upDated == 1){
    //Serial.println("updated");
  //}
  /*
  if(upDated == 1){
    for(int x =0; x<64; x++){
      Serial.print(uartArray[x],HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
    //delay(500);
    //mySerial.write(1);
   */
  //if(updateData(uartArray)==0){  
    //char sensorPacket[18]= {0x11, 0xFF, (myData.EstAlt>>24)&0xFF, (myData.EstAlt>>16)&0xFF, (myData.EstAlt>>8)&0xFF, myData.EstAlt&0xFF, (myData.gyroADC[0]>>8)&0xFF, myData.gyroADC[0]&0xFF, (myData.gyroADC[1]>>8)&0xFF, myData.gyroADC[1]&0xFF, (myData.gyroADC[2]>>8)&0xFF, myData.gyroADC[2]&0xFF, (myData.accADC[0]>>8)&0xFF, myData.accADC[0]&0xFF, (myData.accADC[1]>>8)&0xFF, myData.accADC[1]&0xFF, (myData.accADC[2]>>8)&0xFF, myData.accADC[2]&0xFF};
    //sendPacket2(sensorPacket,0x12);
   /* Serial.print("EstAlt = ");
    Serial.println(myData.EstAlt,DEC);
    
    
    Serial.print("ultraSonic = ");
    Serial.println(myData.ultraSonic,DEC);
    
    Serial.print("myData.gyroADC[0] = ");
    Serial.println(myData.gyroADC[0],DEC);
    Serial.print("myData.gyroADC[1] = ");
    Serial.println(myData.gyroADC[1],DEC);
    Serial.print("myData.gyroADC[2] = ");
    Serial.println(myData.gyroADC[2],DEC);
    
    Serial.print("myData.gyroADC[0] = ");
    Serial.println(myData.accADC[0],DEC);
    Serial.print("myData.gyroADC[1] = ");
    Serial.println(myData.accADC[1],DEC);
    Serial.print("myData.gyroADC[2] = ");
    Serial.println(myData.accADC[2],DEC);*/
  //}
  
  
  if(1==listenForPacket2(recvPacket,0x12)){
    recvData.EstAlt = recvPacket[0];
    recvData.EstAlt = recvData.EstAlt<<8;
    recvData.EstAlt = recvPacket[1];
    recvData.EstAlt = recvData.EstAlt<<8;    
    recvData.EstAlt = recvPacket[2];
    recvData.EstAlt = recvData.EstAlt<<8;
    recvData.EstAlt = recvPacket[3];
    
    recvData.gyroADC[0] = recvPacket[4];
    recvData.gyroADC[0] = recvData.gyroADC[0]<<8;
    recvData.gyroADC[0] = recvPacket[5];
    
    recvData.gyroADC[1] = recvPacket[6];
    recvData.gyroADC[1] = recvData.gyroADC[1]<<8;
    recvData.gyroADC[1] = recvPacket[7];
    
    recvData.gyroADC[2] = recvPacket[8];
    recvData.gyroADC[2] = recvData.gyroADC[2]<<8;
    recvData.gyroADC[2] = recvPacket[9];
    
    recvData.accADC[0] = recvPacket[10];
    recvData.accADC[0] = recvData.accADC[0]<<8;
    recvData.accADC[0] = recvPacket[11];
    
    recvData.accADC[1] = recvPacket[12];
    recvData.accADC[1] = recvData.accADC[1]<<8;
    recvData.accADC[1] = recvPacket[13];
    
    recvData.accADC[2] = recvPacket[14];
    recvData.accADC[2] = recvData.accADC[2]<<8;
    recvData.accADC[2] = recvPacket[15];
    
    
    Serial.print("EstAlt = ");
    Serial.println(recvData.EstAlt,DEC);
    
    Serial.print("recvData.gyroADC[0] = ");
    Serial.println(recvData.gyroADC[0],DEC);
    Serial.print("recvData.gyroADC[1] = ");
    Serial.println(recvData.gyroADC[1],DEC);
    Serial.print("recvData.gyroADC[2] = ");
    Serial.println(recvData.gyroADC[2],DEC);
    
    Serial.print("recvData.gyroADC[0] = ");
    Serial.println(recvData.accADC[0],DEC);
    Serial.print("recvData.gyroADC[1] = ");
    Serial.println(recvData.accADC[1],DEC);
    Serial.print("recvData.gyroADC[2] = ");
    Serial.println(recvData.accADC[2],DEC);
    
    Serial.println("");
  }
  
    Serial.println(listenForPacket2(recvPacket,0x09));
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
  
  //backupData();            //puts current data into another same structure, just in case we need them
  
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
    }
     
    place++;
    if(place>=64){    //if it reach the max buffer size and array size without seeing End Byte, it is cuted off
      //revertData();  //put old data back and return fail
      return 1;  
    }
  }while(array[place]!=endByte);  //loop until seeing endByte
  
  for(int CT; CT<64;CT++){      //reset the array for storing data
    array[CT] = 0;
  }
  mySerial.flush();            //flush everything left in the uart rx buffer, size 64 bytes
  
  return 0;
  
}

int storeData16(int dType, int16_t data16){//determine what type of int 16 data it is, and store it in correct place, will add as we go along
  switch(dType){
    case 1: 
      myData.magADC[0] = data16;
      break;
    case 2:
      myData.magADC[1] = data16;
      break;
    case 3:
      myData.magADC[2] = data16;
      break;
    case 4:
      myData.heading = data16;
      break;    
    case 5: 
      myData.gyroADC[0] = data16;
      break;
    case 6:
      myData.gyroADC[1] = data16;
      break;
    case 7:
      myData.gyroADC[2] = data16;
      break;
    case 8: 
      myData.accADC[0] = data16;
      break;
    case 9:
      myData.accADC[1] = data16;
      break;
    case 10:
      myData.accADC[2] = data16;
      break;
      
    default :
      return 1;
  }
  return 0;
}

int storeData32(int dType, int32_t data32){  //determine what type of int 32 data it is, and store it in correct place, will add as we go along
  switch(dType){
    case 32:
      myData.EstAlt = data32;
      break;
    default :
      return 1;
  }
  return 0;
}


