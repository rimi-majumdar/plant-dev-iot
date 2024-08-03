#define TRIG_PIN 2
#define ECHO_PIN 15

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Trigger the sensor by sending a 10µs pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  int distance_cm = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(1000); // Wait for a while before taking the next measurement
}
