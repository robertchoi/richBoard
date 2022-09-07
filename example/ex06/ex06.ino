void setup() {
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  int knobValue = analogRead(A1);
  // print out the value you read:
  Serial.println(knobValue);
  delay(500);        // delay in between reads for stability
}
