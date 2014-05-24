#include "motorcontrol.h"
#include <Arduino.h>


void setup() 
{
  // put your setup code here, to run once:
  initializePWMs();
  ArmMotors();
  delay(500);
  writeThrust(8);
  delay(500);
  writeThrust(9);
  delay(200);  
  writeThrust(10);
  delay(200);  
  writeThrust(11);
  delay(200);  
  //writeThrust(12);
  //delay(200);  
  //writeThrust(13); 
  //delay(200);  
  //writeThrust(12);   
  //delay(200);  
  //writeThrust(11);
  //delay(200);  
  writeThrust(10);   
  delay(200);  
  writeThrust(9);
  delay(200);  
  writeThrust(8);
  delay(200);  
  writeThrust(7);   
  disarmMotors();
  


}

void loop()
{ 
  
}
