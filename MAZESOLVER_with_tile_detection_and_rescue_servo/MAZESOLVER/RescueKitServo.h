//change connections for servo **

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

  // Opens the cover, lets one kit fall, then closes the cover.
  void dispense() {
    if (!isReady) return;

    servo.write(openPosition);
    delay(releaseTime);
    servo.write(closedPosition);
    delay(300);
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
