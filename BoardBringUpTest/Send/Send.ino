#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

#include <SPI.h>
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F
#define CC2500_SRES    0x30       // reset strobe 

#define PACKET_LENGTH 0X07
#define SEND_FORLOOP 0x08
#define ADDRESS 0xFF

//char TP[] = {7, 0x05, 'H','E','L','L','O','!'}; //packet length (includes address and data), device adress 
void setup(){
  Serial.begin(9600);  
  pinMode(2, OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);  
  //pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(2,HIGH);
  SendStrobe(CC2500_SRES);
  init_CC2500_V2();  
  pinMode(9, OUTPUT);
    
}
void loop()
{    
  sendPacket(0x25,0x35,0x10,0x20,0x12,0x13); 
} 


void sendPacket(byte name, byte target, byte distance, byte sensorData, byte hop, byte end_byte)
{
  char fullPacket[] = {PACKET_LENGTH, ADDRESS, name, target, distance, sensorData, hop, end_byte}; 
  SendStrobe(CC2500_IDLE); 
  SendStrobe(CC2500_FTX);
  WriteTX_burst(CC2500_TXFIFO,fullPacket,SEND_FORLOOP);
  //do not add code between the strobe and while loops otherwise it will miss the conditions !!!!!!!!!!!!!!
  SendStrobe(CC2500_TX); 
  while (!digitalRead(MISO)) { }    
  while (digitalRead(MISO)) { }    
  //do not add code between the strobe and while loops otherwise it will miss the conditions !!!!!!!!!!!!!!
  SendStrobe(CC2500_IDLE); 
}

