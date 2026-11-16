#pragma once

#include <ESP32Servo.h>

/*
 * Separate rescue-kit dispensing class.
 * It does not perform camera detection or control the NeoPixel.
 *
 * Default wiring/angles are starting values:
 *   Servo signal: GPIO 2 (the original sketch's commented dropper pin)
 *   Closed angle: 85 degrees (the original sketch's dropperAngle)
 *   Open angle:   15 degrees (adjust for your mechanism)
 */
class RescueKitServo {
public:
  void begin(int signalPin = 2,
             int closedAngle = 85,
             int openAngle = 15,
             unsigned long openTimeMs = 700) {
    pin = signalPin;
    closedPosition = constrain(closedAngle, 0, 180);
    openPosition = constrain(openAngle, 0, 180);
    releaseTime = openTimeMs;

    servo.setPeriodHertz(50);
    servo.attach(pin, 500, 2400);
    servo.write(closedPosition);
    isReady = true;
    delay(300);
  }

  // Dispenses the requested number of kits, one at a time.
  // Each kit gets a separate open-close cycle.
  void dispense(int kitCount) {
    if (!isReady || kitCount <= 0) return;

    // Keep this dropper's command within the supported victim-kit range.
    kitCount = constrain(kitCount, 0, 2);

    for (int i = 0; i < kitCount; i++) {
      servo.write(openPosition);
      delay(releaseTime);
      servo.write(closedPosition);
      delay(300);
    }
  }

  // Backward-compatible one-kit command.
  void dispense() {
    dispense(1);
  }

  void close() {
    if (isReady) servo.write(closedPosition);
  }

private:
  Servo servo;
  int pin = 2;
  int closedPosition = 85;
  int openPosition = 15;
  unsigned long releaseTime = 700;
  bool isReady = false;
};
