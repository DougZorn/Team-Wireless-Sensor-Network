#ifndef NODE INTERFACE_H_
#define DEF_H_

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"
#include "Serial.h"

extern int16_t sensorDataU;

byte checkNode();
void maintainNode();

#endif
