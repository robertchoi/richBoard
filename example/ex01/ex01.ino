#define LASER_PORT  12

void setup() {
  pinMode(LASER_PORT, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LASER_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LASER_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
