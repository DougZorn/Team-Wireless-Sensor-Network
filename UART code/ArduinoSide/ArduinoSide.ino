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
  mySerial.begin(57600);
}

void loop()
{
  //if (mySerial.available()) Serial.write(mySerial.read());
  //if (Serial.available()) mySerial.write(Serial.read());
  if(mySerial.available()){  // this checks the rx buffer if there is anything there, Buffer_Size = 64Byte
    temp = mySerial.read();  //read one byte of data from buffer to temp
    
    
    //Reading From MultiWii Control Board to Mini
    // this is for assembling the int16_t sent from the buffer. Used for sensor data
    if(x==0){                //bit 8 - 15
      y = temp;              //take the byte
      y = y << 8;            //Shift it up 8 bits to its location
      x++; 
    }else{                  //bit 0 to 7
      y += temp;            //append the bits to 0-7 bit of the 8-15 bit
      Serial.println(y, DEC);  //print to serial in decimals
      y = 0;
      x=0;
    }
    
    //Writing From Mini to MultiWii Control Board
    //this sents one byte of data across the UART
    byte temp;
    mySerial.write(temp);
    
    //same will be done if sending int16 instead of one byte
    //example
    int16_t y;
    
    if(x==0){                  //first send the bit 8-15
      temp = y>>8;  
      mySerial.write(temp);
      x++;
    }else{                    //2nd sent the bit 0-7
      temp = y;
      mySerial.write(temp);
      x=0;
    }
    
  }
  
}

