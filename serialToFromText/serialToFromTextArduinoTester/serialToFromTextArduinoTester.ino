/*  
 *  This is to test the functionality of the serial-text program
 *
 */

#include <SPI.h>
#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

void setup(){
  Serial.begin(9600);
  init_CC2500_V2();
  pinMode(9,OUTPUT);
  
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
  
  Serial.println("-3 17");
  for(int i = 0; i < 10; i++){
    Serial.print("hello world ");
    Serial.println(i);
  }
  Serial.println("-4 17");
  
}

int rounds = -1;
int incomingByte = 0;

void loop(){
  
  
  if(Serial.available() > 0){
    
    incomingByte = Serial.read();

                // say what you got:
                Serial.print("node received: ");
                Serial.println(incomingByte, DEC);
    
  }
  
  
  
  
  /*
  Serial.print("-3 ");
  Serial.println(rounds);
  for(int i = 0; i < 5; i++){
    Serial.println("hello world");
  }
  Serial.print("-4 ");
  Serial.println(rounds);
  
  rounds++;
  delay(1000);
  //noLoop()
  
  */
}
