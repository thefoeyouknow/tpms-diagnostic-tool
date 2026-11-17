// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>

enum class LfWakePattern : uint8_t {
  CW = 0,
  PULSED = 1
};

struct LfWakeEvent {
  LfWakePattern pattern;
  uint32_t      duration_ms;
  bool          is_auto;
  uint32_t      timestamp_ms;
};

class LfWakeDriver {
 public:
  void begin();
  void update(uint32_t nowMs);

  // Trigger a manual or automatic 125 kHz wake burst.
  // Returns false if a burst is already running or cooldown has not elapsed.
  bool triggerBurst(LfWakePattern pattern, uint32_t durationMs = 0, bool isAuto = false);
  void abort();

  void setAutoWake(bool enabled, uint32_t intervalMs = 0);
  bool isAutoWake() const { return _autoEnabled; }
  uint32_t autoIntervalMs() const { return _autoIntervalMs; }

  bool isActive() const { return _state != State::IDLE; }
  uint32_t burstCount() const { return _burstCount; }
  uint32_t cooldownRemainingMs(uint32_t nowMs) const;

  bool popCompletedEvent(LfWakeEvent& out);

 private:
  enum class State : uint8_t {
    IDLE = 0,
    CW_ACTIVE,
    PULSE_ON,
    PULSE_OFF,
    PULSE_TAIL_CW
  };

  void startCarrier();
  void stopCarrier();
  void finishBurst(uint32_t nowMs);

  State         _state = State::IDLE;
  LfWakePattern _activePattern = LfWakePattern::CW;
  bool          _activeIsAuto = false;
  uint8_t       _pulseIndex = 0;
  uint32_t      _phaseStartMs = 0;
  uint32_t      _burstStartMs = 0;
  uint32_t      _targetDurationMs = 0;
  uint32_t      _lastFinishedMs = 0;
  bool          _hasFinishedOnce = false;

  bool          _autoEnabled = false;
  uint32_t      _autoIntervalMs = 5000U;
  uint32_t      _lastAutoTriggerMs = 0;

  uint32_t      _burstCount = 0;
  bool          _hasCompletedEvent = false;
  LfWakeEvent   _completedEvent{};
};
