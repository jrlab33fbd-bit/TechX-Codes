// APDS9960 RGBC registers
static const uint8_t TILE_APDS_ADDRESS = 0x39;
static const uint8_t TILE_ENABLE = 0x80;
static const uint8_t TILE_ATIME = 0x81;
static const uint8_t TILE_CONTROL = 0x8F;
static const uint8_t TILE_CDATAL = 0x94;

void TileDetector::selectMuxChannel() {
  Wire.beginTransmission(0x70); // TCA9548A address used by this project
  Wire.write((uint8_t)(1U << channel));
  Wire.endTransmission();
}

void TileDetector::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TILE_APDS_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void TileDetector::begin(uint8_t muxChannel) {
  channel = muxChannel;
  selectMuxChannel();
  delay(10);

  // 0xDB = about 89 ms RGBC integration time.
  writeReg(TILE_ATIME, 0xDB);
  // RGBC gain = 4x. Proximity gain is not important for RGBC readings.
  writeReg(TILE_CONTROL, 0x01);
  // Power ON + RGBC engine + proximity engine. Keep proximity enabled so
  // this sensor can still provide proximity data if needed.
  writeReg(TILE_ENABLE, 0x07);
  delay(120); // allow the first color integration to complete

  lastSampleMs = millis();
  candidate = TILE_UNKNOWN;
  candidateCount = 0;
  armed = true;
  neutralCount = 0;

  Serial.println("Floor tile detector ready on TCA9548A channel 7.");
  Serial.println("Serial RGBC readings help tune blackClearMax/colorClearMin.");
}

bool TileDetector::readColor(uint16_t &clear, uint16_t &red, uint16_t &green, uint16_t &blue) {
  selectMuxChannel();
  Wire.beginTransmission(TILE_APDS_ADDRESS);
  Wire.write(TILE_CDATAL);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(TILE_APDS_ADDRESS, (uint8_t)8);
  if (received != 8) {
    while (Wire.available()) Wire.read();
    return false;
  }

  uint8_t data[8];
  for (uint8_t i = 0; i < 8; i++) data[i] = Wire.read();

  clear = (uint16_t)data[1] << 8 | data[0];
  red   = (uint16_t)data[3] << 8 | data[2];
  green = (uint16_t)data[5] << 8 | data[4];
  blue  = (uint16_t)data[7] << 8 | data[6];
  return true;
}

TileDetector::TileColor TileDetector::classify(uint16_t clear, uint16_t red,
                                                uint16_t green, uint16_t blue) {
  // A zero reading is treated as invalid/unknown, not as a black tile.
  if (clear == 0) return TILE_UNKNOWN;

  // Check black first: black normally reflects very little light.
  if (clear <= blackClearMax) return TILE_BLACK;

  // Only evaluate blue when enough light is returned for a reliable ratio.
  if (clear >= colorClearMin &&
      (uint32_t)blue * 100U > (uint32_t)red * blueRedPercent &&
      (uint32_t)blue * 100U > (uint32_t)green * blueGreenPercent) {
    return TILE_BLUE;
  }

  return TILE_UNKNOWN;
}

bool TileDetector::performAction(TileColor color) {
  moveMotor(0, 0);
  // Requested behavior: set NeoPixels to black/off for both tile types.
  showColor(strip.Color(0, 0, 0));

  if (color == TILE_BLUE) {
    Serial.println("BLUE TILE: stop, wait 5 seconds, then continue.");
    delay(5000);
    MoveStrightToDistance(actionForwardMm);
  } else if (color == TILE_BLACK) {
    Serial.println("BLACK TILE: perform 180-degree U-turn, then continue.");
    turnBot(180, LEFT);
    delay(100);
    MoveStrightToDistance(actionForwardMm);
  } else {
    return false;
  }

  moveMotor(0, 0);
  // Do not retrigger while the sensor is still over the same colored tile.
  // It becomes armed again after several neutral/non-blue/non-black samples.
  armed = false;
  neutralCount = 0;
  candidate = TILE_UNKNOWN;
  candidateCount = 0;
  return true;
}

bool TileDetector::service() {
  const unsigned long now = millis();
  if (now - lastSampleMs < sampleIntervalMs) return false;
  lastSampleMs = now;

  uint16_t clear = 0, red = 0, green = 0, blue = 0;
  if (!readColor(clear, red, green, blue)) {
    Serial.println("Tile sensor read failed; check APDS9960 and TCA channel 7.");
    candidate = TILE_UNKNOWN;
    candidateCount = 0;
    return false;
  }

  TileColor detected = classify(clear, red, green, blue);
#ifdef DEBUG
  Serial.print("Tile RGBC C:"); Serial.print(clear);
  Serial.print(" R:"); Serial.print(red);
  Serial.print(" G:"); Serial.print(green);
  Serial.print(" B:"); Serial.print(blue);
  Serial.print(" -> ");
  if (detected == TILE_BLUE) Serial.println("BLUE");
  else if (detected == TILE_BLACK) Serial.println("BLACK");
  else Serial.println("OTHER");
#endif

  if (!armed) {
    if (detected == TILE_UNKNOWN) {
      if (neutralCount < 255) neutralCount++;
      if (neutralCount >= 3) {
        armed = true;
        neutralCount = 0;
        Serial.println("Tile detector re-armed.");
      }
    } else {
      neutralCount = 0;
    }
    return false;
  }

  if (detected == TILE_UNKNOWN) {
    candidate = TILE_UNKNOWN;
    candidateCount = 0;
    return false;
  }

  if (detected != candidate) {
    candidate = detected;
    candidateCount = 1;
  } else if (candidateCount < 255) {
    candidateCount++;
  }

  if (candidateCount >= samplesToConfirm) {
    return performAction(candidate);
  }
  return false;
}
