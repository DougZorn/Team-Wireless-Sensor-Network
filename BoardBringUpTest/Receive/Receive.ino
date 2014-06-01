#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

#include <SPI.h>

byte Packet[7]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void setup()
{
  Serial.begin(9600);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  pinMode(2,OUTPUT);
  SPI.begin();
  digitalWrite(2,HIGH);  
  SendStrobe(CC2500_SRES);
  init_CC2500_V2();
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
}

void loop()
{   
  if(listenForPacket(Packet)==0){
    Serial.println("Failed");
  }else{
    Serial.println("Passed");
  }
} 


