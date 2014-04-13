#include "motorcontrol.h"

void setup() 
{
  // put your setup code here, to run once:
  initializePWMs();  
}

void loop()
{
 delay(100);
 writeRudder(8);
 delay(100);
 writeRudder(9);
 delay(100);
 writeRudder(10);
 delay(100);
 writeRudder(11);
 delay(100);
 writeRudder(12);
 delay(100);
 writeRudder(13);
 delay(100);
 writeRudder(14);
 delay(1000);
  
}
