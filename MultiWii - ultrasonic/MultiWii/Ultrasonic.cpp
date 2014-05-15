#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "RX.h"

short sonicValue = 0;
short displayValue = 0;
//int sonicValue = 0;

void sonic_init(){
  pinMode(A0);
}

void sonic_update(){
  sonicValue = analogRead(A1);
  displayValue = 1000 + sonicValue;
  rcData[AUX4] = displayValue;
  
}
