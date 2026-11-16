

#include <Arduino.h>

String received = "";

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial1.begin(115200, SERIAL_8N1, 37, 39);

  Serial.println("LOLIN READY - WAITING FOR NICLA");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      received.trim();

      if (received.indexOf("UNHARMED") >= 0) {
        Serial.println("UNHARMED");
      }
      else if (received.indexOf("HARMED") >= 0) {
        Serial.println("HARMED");
      }
      else if (received.indexOf("STABLE") >= 0) {
        Serial.println("STABLE");
      }

      received = "";
    }
    else if (c >= 32 && c <= 126) {
      if (received.length() < 40) {
        received += c;
      }
    }
  }
}
