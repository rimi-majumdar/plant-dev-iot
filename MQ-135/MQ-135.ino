int analogSensorPin = A0;  // Analog input pin for the sensor
int digitalSensorPin = 14;  // Digital input pin for the sensor

int sensorValue;
int digitalValue;

void setup()
{
  Serial.begin(115200); // sets the serial port to 115200 for ESP32
  pinMode(digitalSensorPin, INPUT);  // use GPIO 14 as INPUT
}

void loop()
{
  sensorValue = analogRead(analogSensorPin); // read analog input pin A0
  digitalValue = digitalRead(digitalSensorPin); // read digital input on GPIO 14
  
  if (sensorValue > 400)
  {
    digitalWrite(digitalSensorPin, HIGH); // set GPIO 14 HIGH
  }
  else
  {
    digitalWrite(digitalSensorPin, LOW); // set GPIO 14 LOW
  }
  
  Serial.print("Analog Value: ");
  Serial.println(sensorValue, DEC); // prints the analog value read
  Serial.print("Digital Value: ");
  Serial.println(digitalValue, DEC); // prints the digital value read
  
  delay(1000); // wait 1000ms for the next reading
}
