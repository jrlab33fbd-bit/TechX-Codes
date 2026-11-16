#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 15;

// Servo positions
const int PHI_POSITION = 30;
const int OMEGA_POSITION = 90;
const int PSI_POSITION = 150;

void setup() {
  Serial.begin(115200);

  // UART1
  // GP5 = RX
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.begin(115200);

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  Serial.println("Pico ready!");
}

void loop() {

  if (Serial2.available()) {

    char command = Serial2.read();

    Serial.print("Received: ");
    Serial.println(command);

    if (command == 'P') {
      Serial.println("PHI detected");
      myServo.write(PHI_POSITION);
      delay(1000);
      myServo.write(90);
    }

    else if (command == 'O') {
      Serial.println("OMEGA detected");
      myServo.write(OMEGA_POSITION);
      delay(1000);
      myServo.write(90);
    }

    else if (command == 'S') {
      Serial.println("PSI detected");
      myServo.write(PSI_POSITION);
      delay(1000);
      myServo.write(90);
    }
  }
}
