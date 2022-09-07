const int buttonPin = 24;     


// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status


void setup() {
  Serial.begin(9600);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    Serial.println("Touch on");
  } else {
    Serial.println("Touch off");
  }  
}
