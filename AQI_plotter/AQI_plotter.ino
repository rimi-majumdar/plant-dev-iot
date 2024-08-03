int Sensor_input = 14;    /*Digital pin 5 for sensor input*/
int airq_input = 0; 
void setup() {
  Serial.begin(115200);  /*baud rate for serial communication*/
}
void loop() 
{
  int airq_input = analogRead(Sensor_input); 
  Serial.print("Variable_1:");
  Serial.println(airq_input);
  delay(100);
}  