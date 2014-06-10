

#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7);   // RX, TX

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
  if(mySerial.available()){
    Serial.write(mySerial.read());
  }
  
}
