#include <SPI.h>
#ifndef READ_WRITE_H
#define READ_WRITE_H

void WriteReg(char addr, char value) //see page 22 of cc2500 data sheet for timing
{
  
  digitalWrite(2,LOW);
  
  while (digitalRead(MISO) == HIGH)
  {
  };    
  SPI.transfer(addr);  
  SPI.transfer(value);  
  digitalWrite(2,HIGH);
}

void WriteTX_burst(char addr, char value[], byte count)
{  
  addr = addr + 0x40;  
  digitalWrite(2,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);

  for(byte i = 0; i<count; i++)
  {
    SPI.transfer(value[i]);
  }
  digitalWrite(2,HIGH);
}

char ReadReg(char addr){
  addr = addr + 0x80;  
  digitalWrite(2,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);  
  char y = SPI.transfer(0);  
  digitalWrite(2,HIGH);
  return y;  
}

char ReadOnly_Reg(char addr){
  addr = addr + 0xC0;
  digitalWrite(2,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);  
  char y = SPI.transfer(0);  
  digitalWrite(2,HIGH);
  return y;  
}

void SendStrobe(char strobe){ 
  digitalWrite(2,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };  
  SPI.transfer(strobe);    
  digitalWrite(2,HIGH);
}

#endif
