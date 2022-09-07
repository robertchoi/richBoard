#define LED_R_PORT  44
#define LED_Y_PORT  45
#define LED_G_PORT  46


void setup() {
  pinMode(LED_R_PORT, OUTPUT);
  pinMode(LED_Y_PORT, OUTPUT);
  pinMode(LED_G_PORT, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_R_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_R_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  
  digitalWrite(LED_Y_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_Y_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  digitalWrite(LED_G_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_G_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
