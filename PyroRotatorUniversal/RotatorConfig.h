#pragma once

#include <Arduino.h>

static constexpr uint32_t ROTATOR_CONFIG_MAGIC = 0x53525232; // "SRR2"
static constexpr uint16_t ROTATOR_CONFIG_VERSION = 1;

enum HomingMode : uint8_t {
  HOME_MANUAL_ALL = 0,     // H records the current AZ and EL as zero
  HOME_AZ_MANUAL_EL_SWITCH = 1,
  HOME_DISABLED = 2
};

struct RotatorConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  int8_t azStepPin, azDirPin;
  int8_t elStepPin, elDirPin;
  int8_t enablePin;
  int8_t azLimitPin, elLimitPin;

  float motorSteps;
  float microsteps;
  float azGearRatio, elGearRatio;
  float azMin, azMax, elMin, elMax;
  float maxSpeedAz, maxSpeedEl; // output degrees/second
  float accelerationAz, accelerationEl; // output degrees/second^2
  float homeSpeed, homeSlowSpeed, homeBackoffDeg;

  bool azDirectionInvert, elDirectionInvert;
  bool enableActiveLow;
  bool azLimitActiveLow, elLimitActiveLow;
  bool allowStrappingPins;
  int8_t elHomeDirection;
  uint8_t homingMode;
  uint8_t reserved[4];
  uint32_t crc;
};

inline RotatorConfig stockConfig() {
  RotatorConfig c{};
  c.magic = ROTATOR_CONFIG_MAGIC;
  c.version = ROTATOR_CONFIG_VERSION;
  c.size = sizeof(RotatorConfig);
  c.azStepPin = 25; c.azDirPin = 26;
  c.elStepPin = 27; c.elDirPin = 14;
  c.enablePin = 13;
  c.azLimitPin = 32; c.elLimitPin = 33;
  c.motorSteps = 200; c.microsteps = 8;
  c.azGearRatio = 7.5f; c.elGearRatio = 4.0f;
  c.azMin = -3600; c.azMax = 3600;
  c.elMin = 0; c.elMax = 90;
  c.maxSpeedAz = 60; c.maxSpeedEl = 90;
  c.accelerationAz = 30; c.accelerationEl = 60;
  c.homeSpeed = 30; c.homeSlowSpeed = 8; c.homeBackoffDeg = 5;
  c.azDirectionInvert = false; c.elDirectionInvert = false;
  c.enableActiveLow = true;
  c.azLimitActiveLow = true; c.elLimitActiveLow = true;
  c.allowStrappingPins = false;
  c.elHomeDirection = -1;
  c.homingMode = HOME_AZ_MANUAL_EL_SWITCH;
  return c;
}

inline uint32_t configCrc(const RotatorConfig &config) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&config);
  const size_t n = offsetof(RotatorConfig, crc);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
