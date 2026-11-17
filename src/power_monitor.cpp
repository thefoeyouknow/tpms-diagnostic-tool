// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include "power_monitor.h"
#include "config.h"

void PowerMonitor::begin() {
  analogReference(AR_DEFAULT);
  analogReadResolution(12);
  analogSampleTime(10);

  // Discard initial dummy conversion after reference/resolution setup
  (void)analogRead(PIN_VBAT_SENSE);
  delay(2);

  // Prime EMA filter with an averaged initial reading
  float sum = 0.0f;
  constexpr uint8_t kInitSamples = 4;
  for (uint8_t i = 0; i < kInitSamples; ++i) {
    sum += readInstantVoltage();
  }
  _filteredV = sum / static_cast<float>(kInitSamples);
  _initialized = true;
  _lastSampleMs = millis();

  _status.vbat_v = _filteredV;
  _status.vbat_mv = static_cast<uint16_t>(_filteredV * 1000.0f + 0.5f);
  _status.soc_percent = voltageToSoc(_filteredV);
  _status.low_battery = (_filteredV <= VBAT_LOW_WARN_V);
}

void PowerMonitor::update(uint32_t nowMs) {
  if (!_initialized) {
    begin();
    return;
  }

  if ((nowMs - _lastSampleMs) < VBAT_SAMPLE_INTERVAL_MS) {
    return;
  }
  _lastSampleMs = nowMs;
  (void)sampleNow();
}

PowerStatus PowerMonitor::sampleNow() {
  if (!_initialized) {
    begin();
    return _status;
  }

  const float rawV = readInstantVoltage();
  // Smooth out transient droop (e.g., BLE TX or LF wake burst)
  constexpr float kAlpha = 0.25f;
  _filteredV = (_filteredV * (1.0f - kAlpha)) + (rawV * kAlpha);

  _status.vbat_v = _filteredV;
  _status.vbat_mv = static_cast<uint16_t>(_filteredV * 1000.0f + 0.5f);
  _status.soc_percent = voltageToSoc(_filteredV);
  _status.low_battery = (_filteredV <= VBAT_LOW_WARN_V);
  return _status;
}

float PowerMonitor::readInstantVoltage() {
  // Average 4 quick SAADC readings to reduce switching noise
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    acc += analogRead(PIN_VBAT_SENSE);
  }
  const float avgCounts = static_cast<float>(acc) * 0.25f;
  return (avgCounts * VBAT_DIVIDER_RATIO * VBAT_ADC_REF_V) / VBAT_ADC_MAX_COUNTS;
}

uint8_t PowerMonitor::voltageToSoc(float vbat) {
  if (vbat <= VBAT_MIN_V) {
    return 0;
  }
  if (vbat >= VBAT_MAX_V) {
    return 100;
  }
  const float norm = (vbat - VBAT_MIN_V) / (VBAT_MAX_V - VBAT_MIN_V);
  const int pct = static_cast<int>(norm * 100.0f + 0.5f);
  return static_cast<uint8_t>(constrain(pct, 0, 100));
}
