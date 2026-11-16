#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 15;  // GP15

void setup() {
  myServo.attach(SERVO_PIN);

  // Start at center
  
  myServo.write(90);
  delay(1000);
}

void loop() {

  // Move to 0 degrees
  myServo.write(0);
  delay(1000);

  // Move to 90 degrees
  myServo.write(90);
  delay(1000);

  // Back to center
  myServo.write(90);
  delay(1000);
}
