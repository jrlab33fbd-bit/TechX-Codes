
#include <Arduino.h>
#include <ESP32Servo.h>

Servo testServo;

const int SERVO_PIN = 12;

void setup() {
  Serial.begin(115200);
  delay(1000);

  testServo.setPeriodHertz(50);
  testServo.attach(SERVO_PIN, 500, 2500);

  Serial.println("SERVO TEST STARTED");
}

void loop() {
  Serial.println("Position: 0 degrees");
  testServo.write(0);
  delay(1500);

  Serial.println("Position: 90 degrees");
  testServo.write(90);
  delay(1500);

  Serial.println("Position: 180 degrees");
  testServo.write(180);
  delay(1500);

  Serial.println("Returning to 0 degrees");
  testServo.write(0);
  delay(1500);
}
