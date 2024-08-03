int signalPin = 16;
void setup() {
   // put your setup code here, to run once:
   Serial.begin(9600);
   pinMode(signalPin,INPUT);
}
void loop() {
   // put your main code here, to run repeatedly:
   if (digitalRead(signalPin) == HIGH) {
      Serial.println("Obstacle present right now!");
   } else {
      Serial.println("No obstacle!");
   }
   delay(1000);
}