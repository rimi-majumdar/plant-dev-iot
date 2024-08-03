// button ONOFF

const int LED = 13;

const int BUTTON = 7; // the input pin where the
// pushbutton is connected
int val = 0; // val will be used to store the state
// of the input pin
void setup() {
pinMode(LED, OUTPUT); // tell Arduino LED is an output
pinMode(BUTTON, INPUT); // and BUTTON is an input
}
void loop(){
val = digitalRead(BUTTON); // read input value and store it
// check whether the input is HIGH (button pressed)
if (val == HIGH) {
digitalWrite(LED, HIGH); // turn LED ON
} else {
digitalWrite(LED, LOW);
}
}


//KEEP LIGHT

const int LED = 13;

const int BUTTON = 7; // the input pin where the
// pushbutton is connected
int val = 0; // val will be used to store the state
// of the input pin
int state = 0; // 0 = LED off while 1 = LED on
void setup() {
pinMode(LED, OUTPUT); // tell Arduino LED is an output
pinMode(BUTTON, INPUT); // and BUTTON is an input
}
void loop() {
val = digitalRead(BUTTON); // read input value and store it
// check if the input is HIGH (button pressed)
// and change the state
if (val == HIGH) {
state = 1 - state;
}
if (state == 1) {
digitalWrite(LED, HIGH); // turn LED ON
} else {
digitalWrite(LED, LOW);
}
}
