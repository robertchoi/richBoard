int speakerPin = 7;
int numTones = 8;
int tones[] = {261, 277, 294, 311, 330, 349, 370, 392};

void setup() {
}


void loop() {
  for(int i = 0; i < numTones; i++)
  {
    tone(speakerPin, tones[i]);
    delay(500);
  }
  noTone(speakerPin);
  delay(1000);
}
