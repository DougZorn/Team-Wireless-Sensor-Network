#include "motorcontrol.h"
#include <Arduino.h>
#include <SoftwareSerial.h>      //needed for SoftwareSerial


SoftwareSerial mySerial(8, 7);   // RX, TX

//Variables for mode
const boolean mode_on = true;
const boolean mode_off = false;

boolean horizonMode_Master;    //case 1
boolean baroMode_Master;      //case 2
boolean magMode_Master;        //case 3

boolean horizonMode_Control;
boolean baroMode_Control;
boolean magMode_Control;

byte readMaster_Mode(int modeID, char* arg){  //read mode that master wants to be in
  switch(modeID){
    case 1:                   //horizonMode
      if(!strcmp(arg, "on" )){
        return 0x21;
      }else if(!strcmp(arg, "off" )){
        return 0x20;
      }else{
        if(horizonMode_Master){
          return 0x21;
        }else{
          return 0x20;
        }
      }
    case 2:                  //baroMode
      if(!strcmp(arg, "on" )){
        return 0x2B;
      }else if(!strcmp(arg, "off" )){
        return 0x2A;
      }else{
        if(baroMode_Master){
          return 0x2B;
        }else{
          return 0x2A;
        }
      }
    case 3:                  //magMode
      if(!strcmp(arg, "on" )){
        return 0x35;
      }else if(!strcmp(arg, "off" )){
        return 0x34;
      }else{
        if(magMode_Master){
          return 0x35;
        }else{
          return 0x34;
        }
      }
    default :
      return 0x01;    //not valid statement s returns 0
  }
}

int changeMode(byte ackType){  //used when ack and or when trying to change mode
  switch(ackType){
    case 0x1E: //horizon
      horizonMode_Control = mode_off;
      break;
    case 0x1F:
      horizonMode_Control = mode_on;
      break;
    case 0x20: 
      horizonMode_Master = mode_off;
      break;
    case 0x21:
      horizonMode_Master = mode_on;
      break;
    case 0x28: //baro 
      baroMode_Control = mode_off;
      break;
    case 0x29:
      baroMode_Control = mode_on;
      break;
    case 0x2A: 
      baroMode_Master = mode_off;
      break;
    case 0x2B:
      baroMode_Master = mode_on;
      break;
    case 0x32: //mag
      magMode_Control = mode_off;
      break;
    case 0x33:
      magMode_Control = mode_on;
      break;
    case 0x34: 
      magMode_Master = mode_off;
      break;
    case 0x35:
      magMode_Master = mode_on;
      break;
    default :
      return 1;
  }
  return 0;
}

int modeAdjust(){
  if(horizonMode_Master != horizonMode_Control){    //using else if because we want to assert one at a time
    //send turn on and off depending on what it is, - 0x02 because master mode on off is always +2 in byte compare to control
    mySerial.write((readMaster_Mode(1, "NULL") - 0x02));    
  }
  if(baroMode_Master != baroMode_Control){ 
    mySerial.write((readMaster_Mode(2, "NULL") - 0x02)); 
  }
  
  if(magMode_Master != magMode_Control){
    mySerial.write((readMaster_Mode(3, "NULL") - 0x02)); 
  }
}

void setup() 
{

  
  mySerial.begin(9600);
  
  changeMode(readMaster_Mode(1,"on"));
  modeAdjust();
  
  // put your setup code here, to run once:
  initializePWMs();
  ArmMotors();  
  delay(500);
  writeThrust(9);
  delay(200);  
  writeThrust(10);
  delay(200);  
  writeThrust(11);
  delay(100);  
  
  changeMode(readMaster_Mode(2,"on"));
  modeAdjust();
  delay(500);
  changeMode(readMaster_Mode(2,"off"));
  modeAdjust();
  /*writeThrust(11);
  delay(200);  
  writeThrust(12);
  delay(200);  
  writeThrust(13); 
  delay(200);  
  writeThrust(12);   
  delay(200);  
  writeThrust(11);
  delay(200);  */
  writeThrust(10);   
  delay(200);  
  writeThrust(9);
  delay(200);  
  writeThrust(8);
  delay(200);  
  writeThrust(7);   
  //disarmMotors();
  
  changeMode(readMaster_Mode(1,"off"));
  modeAdjust();
}

void loop()
{ 
  
}
