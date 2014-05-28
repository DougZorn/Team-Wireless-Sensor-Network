
byte temp;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  if(Serial.available()){
    while(Serial.available()>0){
      temp = Serial.read();
      
    }
  }
  
  Serial.print("Got mesg\n");
}
