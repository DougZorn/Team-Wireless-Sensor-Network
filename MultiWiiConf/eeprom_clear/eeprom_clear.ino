/*
 * EEPROM Clear
 *
 * Sets all of the bytes of the EEPROM to 0.
 * This example code is in the public domain.

 */

#include <EEPROM.h>

void setup()
{
  pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
    
  // turn the LED on when we're done
  //digitalWrite(13, HIGH);
  PORTB |= (1<<7); PORTC |= (1<<7);
}

void loop()
{
}
