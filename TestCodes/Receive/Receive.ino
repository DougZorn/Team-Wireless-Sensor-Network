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
#include "read_write.h"
//#include "read_writeV2.h"
#include "motorcontrol.h"

byte recvPacket[64]= {0x00};
byte Check = 0;
  
void setup()
{ 
  Serial.begin(9600); 
  init_CC2500_V2();  
  
  Serial.println("After SPI Init");
  
  Serial.println(ReadReg(REG_IOCFG0),HEX);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  Serial.println(ReadReg(REG_IOCFG2),HEX);
}

void loop(){
    if(listenForPacket2(recvPacket,0x40)){
      for(int x =0; x<64; x++){
       Serial.write(recvPacket[x]);
       Serial.print(" "); 
      }
       Serial.println(" "); 
    }
    
} 



