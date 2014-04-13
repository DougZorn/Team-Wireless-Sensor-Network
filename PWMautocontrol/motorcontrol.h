#include <Arduino.h>

#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLS_H


void initializePWMs()
{
  pinMode(10,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  
  
  // Note TCCR0 and TCCR1 although look similar, the registers are configured slightly differently. See datasheet.	
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); //pins 5 and 6
  TCCR0B = _BV(CS02) | _BV(CS00);
	
  TCCR1A =  _BV(COM1A1) | _BV(COM1B1)| _BV(WGM10);//pins 9 and 10
  TCCR1B =  _BV(WGM12) | _BV(CS12) | _BV(CS10);	
	
 /*
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22) | _BV(CS20); // 30.5 Hz
*/  
  OCR0A = 7;  //min
  OCR0B = 7;  //min
  //OCR2B = 7;  //min
  OCR1A = 7;  //min
  OCR1B = 7;  //min
}

//range of value is 7 to 14. Neutral is 10.

void writeRoll(byte value) //pin 9
{  
  OCR1A = value;    
}

void writeThrust(byte value) //pin 10
{
  OCR1B = value;  
}

void writeRudder(byte value) //pin 5
{
  OCR0B = value;  
}

void writePitch(byte value) //pin 6
{  
  OCR0A = value;  
}



#endif
