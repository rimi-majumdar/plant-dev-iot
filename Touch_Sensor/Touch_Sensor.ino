void setup() {
  // put your setup code here, to run once:
  // we write here that block of code which we want to repeat 
 pinMode(2,INPUT);
 pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // checking value 1 or 0 recursively i.e again and again 
digitalRead(2); // read the value at 2
int touchValue = digitalRead(2); // storing the value 
if (touchValue ==1)
{
  digitalWrite(13,HIGH); // led ON
}
else
{
  digitalWrite(13,LOW);
}

}
