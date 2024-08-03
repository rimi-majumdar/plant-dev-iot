const int OUT_PIN = 18;
const int SAMPLE_TIME = 1;
unsigned long millisCurrent;
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;

int sampleBufferValue = 0;
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  millisCurrent = millis();
  millisElapsed = millisCurrent - millisLast;
  if(digitalRead(OUT_PIN)== LOW)
  {
    sampleBufferValue++;
  }
  if(millisElapsed > SAMPLE_TIME)
  {
  Serial.println(sampleBufferValue);
  millisLast = millisCurrent;
  }
}



