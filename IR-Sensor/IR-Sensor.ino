#define SensorPin A0    // Ph Sensor      // the pH meter Analog output is connected with the Arduino’s Analog
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
int relay_pin = 7; // relay
int ir_Sensor = 2; // connect ir sensor module to arduino pin 2
int buzzer = 8; // connect buzzer to arduino pin 8
int trig = 7; // sending signals from ultrasonic sensoe 
int echo = 6;
long timeInMicro; //ultrasonic sensor
int distanceInCm;
const int OUT_PIN = 8;
const int SAMPLE_TIME = 10;
unsigned long millisCurrent;  // sound sensor 
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;
int sensorValue; // MQ 135
int digitalValue;
int signalPin = 7; // proximity sensor 



int sampleBufferValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // start serial communication at 9600 baud rate
  pinMode(ir_Sensor,INPUT);
  pinMode (buzzer , OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(6,INPUT);
  pinMode(13,OUTPUT);  
  Serial.println("Ready");
  pinMode(relay_pin,OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
   pinMode(signalPin,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 int SensorStatus = digitalRead(ir_Sensor);
 if (SensorStatus == 0)
 {
   digitalWrite(buzzer,HIGH);
   //serial.println("An Object is Detected");
 }

else
 
{
digitalWrite(buzzer,LOW);

}
delay(5000);

{
//to recieve
digitalWrite(trig,LOW);
delayMicroseconds(2);
digitalWrite(trig,HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);

//to recieve

timeInMicro = pulseIn(echo,HIGH); // when recieved its high 

// another variable to store 

//speed of sound: 340m/s = 29microseconds/cm
distanceInCm = timeInMicro / 29 /2;

Serial.println(distanceInCm);
delay(100);
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
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
  avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");
  digitalWrite(13, HIGH);       
  delay(800);
  digitalWrite(13, LOW);  
  //relay
  digitalWrite(relay_pin,HIGH);
  delay(2000);
  digitalWrite(relay_pin,LOW);
  delay(2000);  
  sensorValue = analogRead(0); // read analog input pin 0
  digitalValue = digitalRead(2);
  if (sensorValue > 400)
  {
    digitalWrite(13, HIGH);
  }
  else
    digitalWrite(13, LOW);
  Serial.println(sensorValue, DEC); // prints the value read
  Serial.println(digitalValue, DEC);
  delay(1000); // wait 100ms for next reading 
  // proximity sensor 
  if (digitalRead(signalPin) == HIGH) {
      Serial.println("Obstacle present right now!");
   } else {
      Serial.println("No obstacle!");
   }
   delay(1000);

}


}
}

