
int sensorPin = A1;  //dip 7  // select the input pin for the potentiometer
int selectPin = 0;      // select the pin for the LED
int sensorValue[5] = {0};  // variable to store the value coming from the sensor
int sensorAverage = 0;



void setup() {
  pinMode(selectPin, OUTPUT);
}

void loop() {
  sensorAverage = 0;

  for(int i = 0; i < 5;i++)
  {   
    sensorValue[i] = analogRead(sensorPin);
    delay(1);
    sensorAverage = sensorAverage + sensorValue[i];  
  }
  
  //sensorAverage = analogRead(sensorPin);
  sensorAverage = sensorAverage/5; 
  if(sensorAverage > 700)
  digitalWrite(selectPin, HIGH);
  else 
  digitalWrite(selectPin, LOW); 
  
  
}
