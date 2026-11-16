
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial1.begin(115200, SERIAL_8N1, 37, 39);
  Serial1.setTimeout(100);

  Serial.println("LOLIN READY - WAITING FOR NICLA");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
}
