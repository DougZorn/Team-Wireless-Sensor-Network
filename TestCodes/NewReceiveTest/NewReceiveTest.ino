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

byte Packet[7]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void setup()
{ 
  Serial.begin(9600); 
  init_CC2500_V2();  
}

void loop()
{   
  if(1==listenForPacket(Packet)){
    for(int x =0; x<7;x++){
     Serial.println(Packet[x],HEX); 
    }
     Serial.println("");
  }
} 

