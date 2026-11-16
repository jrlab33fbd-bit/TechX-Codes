#ifndef TILE_DETECTOR_H
#define TILE_DETECTOR_H

#include <Arduino.h>

// APDS9960 floor-tile detector connected through a TCA9548A I2C mux.
// The sensor is read directly so this class can coexist with the project's
// existing proximity-sensor implementation.
class TileDetector {
public:
  void begin(uint8_t muxChannel);
  // Call repeatedly from wallFollow(). Returns true when it handled a tile.
  bool service();

  // Tune these after checking the raw RGBC values printed to Serial.
  uint16_t blackClearMax = 180;
  uint16_t colorClearMin = 250;
  uint8_t blueRedPercent = 125;   // Blue must be > red * 1.25
  uint8_t blueGreenPercent = 110; // Blue must be > green * 1.10
  uint8_t samplesToConfirm = 2;
  uint16_t sampleIntervalMs = 100;
  float actionForwardMm = 60.0;   // short move after either tile action

private:
  enum TileColor : uint8_t { TILE_UNKNOWN, TILE_BLUE, TILE_BLACK };
  uint8_t channel = 7;
  unsigned long lastSampleMs = 0;
  TileColor candidate = TILE_UNKNOWN;
  uint8_t candidateCount = 0;
  bool armed = true;
  uint8_t neutralCount = 0;

  bool readColor(uint16_t &clear, uint16_t &red, uint16_t &green, uint16_t &blue);
  TileColor classify(uint16_t clear, uint16_t red, uint16_t green, uint16_t blue);
  void selectMuxChannel();
  void writeReg(uint8_t reg, uint8_t value);
  bool performAction(TileColor color);
};

#endif
