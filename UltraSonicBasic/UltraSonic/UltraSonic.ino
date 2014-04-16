int UltraSonicPin = A0;    // select the input pin for the potentiometer
int SonicValue = 0;  // variable to store the value coming from the sensor

void setup() {
}

void loop() {
  // read the value from the sensor:
  SonicValue = analogRead(UltraSonicPin);
}
