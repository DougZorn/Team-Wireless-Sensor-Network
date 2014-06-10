import processing.serial.*;

void setup() {
  println(Serial.list());
  // Open the port you are using at the rate you want: 
  //put Serial.list()[here] to 1 when no node is plugged in, 2 when there is a node
  myPort = new Serial(this, Serial.list()[2], 9600);                          //Pick Arduino serial port
  myPort.clear();
}

void draw(){
  
  
  
  
  
  
}