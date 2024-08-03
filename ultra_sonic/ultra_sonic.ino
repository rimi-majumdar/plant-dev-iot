#define TRIG_PIN 2
#define ECHO_PIN 15

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH);

  // Speed of sound in air: 343 m/s = 0.0343 cm/µs at 20 degrees Celsius
  // Distance = (Time × Speed of Sound) / 2
  float distance_cm = (duration * 0.0343) / 2.0;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(1000);
}
