// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>

struct PowerStatus {
  float    vbat_v;
  uint16_t vbat_mv;
  uint8_t  soc_percent;
  bool     low_battery;
};

class PowerMonitor {
 public:
  void begin();
  void update(uint32_t nowMs);
  PowerStatus sampleNow();
  PowerStatus status() const { return _status; }

 private:
  float readInstantVoltage();
  static uint8_t voltageToSoc(float vbat);

  PowerStatus _status{0.0f, 0, 0, false};
  float       _filteredV = 0.0f;
  bool        _initialized = false;
  uint32_t    _lastSampleMs = 0;
};
