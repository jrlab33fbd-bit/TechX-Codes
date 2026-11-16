#include <Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);

  // Nicla TX -> Pico GP5
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.begin(115200);

  // Servo signal -> GP15
  myServo.attach(15);
  myServo.write(90);

  Serial.println("Pico ready");
}

void loop() {
  if (Serial2.available()) {
    char command = Serial2.read();

    Serial.print("Received: ");
    Serial.println(command);

    if (command == 'P') {
      myServo.write(30);
      delay(1000);
      myServo.write(90);
    }

    if (command == 'O') {
      myServo.write(90);
      delay(1000);
      myServo.write(90);
    }

    if (command == 'S') {
      myServo.write(150);
      delay(1000);
      myServo.write(90);
    }
  }
}
