#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DHTPIN 4      // Define the GPIO pin for the DHT11 sensor
#define DHTTYPE DHT11 // Specify the type of the DHT sensor

DHT dht(DHTPIN, DHTTYPE); // Create a DHT object

#define SensorPin A0          // the pH meter Analog output is connected with the Arduino’s Analog
unsigned long int avgValue;   // Store the average value of the sensor feedback
float b;
int buf[10], temp;

const int flowSensorPin = 5; // GPIO pin for the flow sensor
volatile int NumPulses;      // Variable for the number of pulses received
float factor_conversion = 7.5; // To convert from frequency to flow rate
float volume = 0;
long dt = 0;  // Time variation for each loop
long t0 = 0;  // millis() from the previous loop

const int trigPin = 2;   // Trigger pin for the ultrasonic sensor
const int echoPin = 15;  // Echo pin for the ultrasonic sensor

const int OUT_PIN = 13;   // Change to GPIO 13 for pH sensor
const int SAMPLE_TIME = 10;

unsigned long millisCurrent;
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;

int sampleBufferValue = 0;

int analogSensorPin = A0;  // Analog input pin for the sensor
int digitalSensorPin = 14; // Digital input pin for the sensor

int sensorValue;
int digitalValue;

const int relayPin = 12;   // Change to the GPIO pin where your relay is connected
const int mq135Pin = 14;   // Change to the Analog pin where your MQ-135 sensor is connected

void setup() {
  Serial.begin(9600);

  // Setup for pH sensor
  pinMode(OUT_PIN, OUTPUT);  // Change to GPIO 13
  Serial.println("pH Sensor Ready");

  // Setup for DHT sensor
  dht.begin();
  delay(2000);  // Add a delay after DHT.begin()

  // Setup for flow sensor
  pinMode(flowSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), PulseCount, RISING);

  // Setup for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setup for relay sensor
  pinMode(relayPin, OUTPUT);

  // Setup for MQ-135 sensor
  pinMode(mq135Pin, INPUT);
}

void loop() {
  // pH Sensor
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(SensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++)
    avgValue += buf[i];
  float phValue = (float)avgValue * 5.0 / 1024 / 6;
  phValue = 3.5 * phValue;
  Serial.print("pH: ");
  Serial.println(phValue, 2);

  // Flow Sensor
  float frequency = GetFrequency();
  float flow_L_m = frequency / factor_conversion;
  dt = millis() - t0;
  t0 = millis();
  volume = volume + (flow_L_m / 60) * (dt / 1000);

  Serial.print("Flow: ");
  Serial.print(flow_L_m, 3);
  Serial.print(" L/min\tVolume: ");
  Serial.print(volume, 3);
  Serial.println(" L");

  // Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long timeInMicro = pulseIn(echoPin, HIGH);
  int distanceInCm = timeInMicro / 29 / 2;
  Serial.println("Distance: " + String(distanceInCm) + " cm");

  // Digital and Analog Sensors
  sensorValue = analogRead(analogSensorPin);
  digitalValue = digitalRead(digitalSensorPin);
  if (sensorValue > 400) {
    digitalWrite(digitalSensorPin, HIGH);
  } else {
    digitalWrite(digitalSensorPin, LOW);
  }

  Serial.print("Analog Value: ");
  Serial.println(sensorValue, DEC);
  Serial.print("Digital Value: ");
  Serial.println(digitalValue, DEC);

  // DHT Sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (!isnan(humidity) && !isnan(temperature)) {
    Serial.print("DHT - Humidity (%): ");
    Serial.println(humidity, 2);
    Serial.print("DHT - Temperature (C): ");
    Serial.println(temperature, 2);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  // Relay Sensor
  int relayValue = digitalRead(relayPin);
  Serial.print("Relay Status: ");
  Serial.println(relayValue);

  // MQ-135 Sensor
  int mq135Value = analogRead(mq135Pin);
  Serial.print("MQ-135 Sensor Value: ");
  Serial.println(mq135Value);

  delay(2000);
}

// Function executed in interrupt
void PulseCount() {
  NumPulses++;
}

// Function to obtain pulse frequency
int GetFrequency() {
  int frequency;
  NumPulses = 0;
  interrupts();
  delay(1000);
  noInterrupts();
  frequency = NumPulses;
  return frequency;
}
