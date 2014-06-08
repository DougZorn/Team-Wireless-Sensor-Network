#include <SPI.h>
#include "cc2500_VAL_V2.h"
#include "cc2500_REG_V2.h"
#ifndef READ_WRITE_H
#define READ_WRITE_H

#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F
#define CC2500_TXFIFO_BURST  0x7F
#define CC2500_RXFIFO_BURST  0xFF
#define CC2500_SRES    0x30       // reset strobe 

#define TX_TIMEOUT 100 // in milliseconds
long previousTXTimeoutMillis = 0;

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

int listenForPacket(byte recvPacket[]) {
  previousTXTimeoutMillis = millis();
  SendStrobe(CC2500_RX);  
  while(!digitalRead(MISO))   
  {
    if(millis()-previousTXTimeoutMillis > TX_TIMEOUT){
     return 0; 
    }
  }
  while(digitalRead(MISO)) 
  {
     if(millis()-previousTXTimeoutMillis > TX_TIMEOUT){
       return 0; 
     }
  } 
  if(ReadOnly_Reg(0x3B)==0)
  {    
    return 0;    
  }
  else
  {
    ReadReg(CC2500_RXFIFO);
    ReadReg(CC2500_RXFIFO);
    for(int i = 0; i < 7; i++)
    {
      recvPacket[i] = ReadReg(CC2500_RXFIFO);     
      Serial.println(recvPacket[i],HEX);
    }    
    Serial.println("");
    
  }  
  SendStrobe(CC2500_IDLE);  
  SendStrobe(CC2500_FRX);
  return 1;
}

#endif
