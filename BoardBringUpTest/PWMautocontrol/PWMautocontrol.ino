#include "motorcontrol.h"
#include <Arduino.h>


void setup() 
{
  // put your setup code here, to run once:
  initializePWMs();
  ArmMotors();  
  delay(100);
  writeThrust(9);
  delay(100);  
  writeThrust(10);
  delay(100);  
  writeThrust(11);
  delay(175);  
  writeThrust(10);
  delay(125);
  writeThrust(9);
  delay(200);  
  writeThrust(8);  
  delay(200);  
  writeThrust(7);  
}

void loop()
{ 
  
}
