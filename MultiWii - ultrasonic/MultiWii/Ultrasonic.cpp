#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "RX.h"

// variables to take x number of readings and then average them
// to remove the jitter/noise from the DYP-ME007 sonar readings
const int numOfReadings = 10; // number of readings to take/ items in the array
int sonicRead[numOfReadings]; // stores the distance readings in an array
int arrayIndex = 0; // arrayIndex of the current item in the array
int sonicTotal = 0; // stores the cumlative total
int averageSonic = 0; // stores the average value

short displayValue = 0;

void sonic_init(){
  pinMode(A0, INPUT);
    // create array loop to iterate over every item in the array
  for (int thisReading = 0; thisReading < numOfReadings; thisReading++) {
    sonicRead[thisReading] = 0;
  }
}

void sonic_update(){

  sonicTotal= sonicTotal - sonicRead[arrayIndex]; // subtract the last distance
  sonicRead[arrayIndex] = analogRead(A0); // add distance reading to array
  sonicTotal= sonicTotal + sonicRead[arrayIndex]; // add the reading to the total
  arrayIndex = arrayIndex + 1; // go to the next item in the array
  // At the end of the array (10 items) then start again
  if (arrayIndex >= numOfReadings) {
    arrayIndex = 0;
  }
  averageSonic = sonicTotal / numOfReadings; // calculate the average distance
  
  displayValue = 1000 + (averageSonic); ///0.0033); //Formats signal reading to display on RX bars in centimeters
  rcData[AUX4] = displayValue; 
  
}
