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


void setup()  
{
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  Serial.begin(9600);
  
  // set the data rate for the SoftwareSerial port, max at 57600
  mySerial.begin(9600);
}

void loop()
{
  //if (mySerial.available()) Serial.write(mySerial.read());
  //if (Serial.available()) mySerial.write(Serial.read());
  if(mySerial.available()){  // this checks the rx buffer if there is anything there, Buffer_Size = 64Byte
    temp = mySerial.read();  //read one byte of data from buffer to temp
    Serial.println(temp);
  }
  
  if(++x>10){
    delay(1000);
    temp = 0x20;
    mySerial.write(temp);
    x=0;
  }
}

