#include <IRremote.h>

int Recv_pin = 13; 
IRrecv irrecv(Recv_pin); 
decode_results results; 

void setup()
{
  Serial.begin(9600);
  Serial.println("Start");
  IrReceiver.begin(Recv_pin, ENABLE_LED_FEEDBACK); 
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX); 
    Serial.println(results.decode_type); 
    irrecv.resume();
  }
}
