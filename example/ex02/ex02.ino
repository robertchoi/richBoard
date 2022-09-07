#define LED_R_PORT  11
#define LED_G_PORT  10
#define LED_B_PORT  9


void setup() {
  pinMode(LED_R_PORT, OUTPUT);
  pinMode(LED_G_PORT, OUTPUT);
  pinMode(LED_B_PORT, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_R_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_R_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  
  digitalWrite(LED_G_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_G_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  digitalWrite(LED_B_PORT, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_B_PORT, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
