const int buttonRedPin = 22;     
const int buttonBluePin = 23;
const int ledRedPin =  11;      
const int ledBluePin =  9;


// variables will change:
int buttonRedState = 0;         // variable for reading the pushbutton status
int buttonBlueState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonRedPin, INPUT);
  pinMode(buttonBluePin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonRedState = digitalRead(buttonRedPin);
  buttonBlueState = digitalRead(buttonBluePin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonRedState == HIGH) {
    // turn LED on:
    digitalWrite(ledRedPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledRedPin, LOW);
  }

  if (buttonBlueState == HIGH) {
    // turn LED on:
    digitalWrite(ledBluePin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledBluePin, LOW);
  }

  
}
