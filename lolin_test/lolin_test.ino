
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial1.begin(115200, SERIAL_8N1, 37, 39);

  Serial.println("LOLIN UART LOOPBACK TEST");
}

void loop() {
  Serial1.println("HELLO FROM LOLIN");
  delay(200);

  while (Serial1.available()) {
    String received = Serial1.readStringUntil('\n');
    received.trim();

    Serial.print("LOOPBACK RECEIVED: ");
    Serial.println(received);
  }

  delay(1000);
}
