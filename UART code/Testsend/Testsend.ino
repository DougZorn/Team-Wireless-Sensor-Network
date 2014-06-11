#include <SoftwareSerial.h>      //needed for SoftwareSerial


//  .begin(baud rate)  starts the UART serial
//  .available()      checks the rx buffer, 0 if there is nothing
//  .print, .println  used for pc reading formate, will create formate bits, Should Only Be Used to Communicate with PC
                      //#NOTE: do not use this for UART transfer between Control Board and Mini

//  .write            uses the bits and write the stuff in ASSCI, 
                      //#NOTE: Use this for sending data between Control Board and Mini
                      
//Note:  Do Not Spam the RX buffere or TX buffer on the Mini or the Control Board
//       it will generate error. 


SoftwareSerial mySerial(8, 7);   // RX, TX
byte temp;                       //For sending or receiving, UART only sents one bytes
int x = 0;                      
int16_t y = 0;                   //for printing to screen
int rounds =0;

void setup()  
{
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop(){
  
    mySerial.write(0x80);
  
    mySerial.write(1);
    temp = 0xFFDB>>8;  
    //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = 0xFFDB;
    //Serial.print(temp,HEX);
    //Serial.print(" ");
    mySerial.write(temp);
    
    mySerial.write(2);
    temp = 0x2233>>8;  
    //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = 0x2211;
    //Serial.print(temp,HEX);
    //Serial.print(" ");
    mySerial.write(temp);
    
    mySerial.write(3);
    temp = 0x1BCD>>8; 
   //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = 0x1BCD;
    //Serial.print(temp,HEX);
    //Serial.print(" ");
    mySerial.write(temp);
    
    mySerial.write(32);
    temp = 0xF2345678 >> 24;
    //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = (0xF2345678>>16);
    //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = (0xF2345678>>8);
    //Serial.print(temp,HEX);
    mySerial.write(temp);
    temp = 0xF2345678;
    //Serial.print(temp,HEX);
    //Serial.print(" ");
    mySerial.write(temp);
    //Serial.println(" ");
    mySerial.write(0xC0);
    
    delay(100);
}

