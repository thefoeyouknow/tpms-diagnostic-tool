#include "lf_wake.h"
#include "config.h"

void LfWakeDriver::begin() {
  // Guarantee MOSFET gate is strictly grounded before configuring PWM
  pinMode(PIN_LF_WAKE_PWM, OUTPUT);
  digitalWrite(PIN_LF_WAKE_PWM, LOW);

  HwPWM2.addPin(PIN_LF_WAKE_PWM);
  HwPWM2.setClockDiv(LF_PWM_CLOCK_DIV);
  HwPWM2.setMaxValue(LF_PWM_COUNTERTOP);
  HwPWM2.stop();
  digitalWrite(PIN_LF_WAKE_PWM, LOW);

  _state = State::IDLE;
  _autoEnabled = false;
  _autoIntervalMs = LF_DEFAULT_AUTO_INTERVAL_MS;
}

void LfWakeDriver::startCarrier() {
  HwPWM2.writePin(PIN_LF_WAKE_PWM, LF_PWM_DUTY_50PCT, false);
}

void LfWakeDriver::stopCarrier() {
  HwPWM2.writePin(PIN_LF_WAKE_PWM, 0, false);
  HwPWM2.stop();
  pinMode(PIN_LF_WAKE_PWM, OUTPUT);
  digitalWrite(PIN_LF_WAKE_PWM, LOW);
}

bool LfWakeDriver::triggerBurst(LfWakePattern pattern, uint32_t durationMs, bool isAuto) {
  const uint32_t nowMs = millis();

  if (_state != State::IDLE) {
    return false;
  }

  if (cooldownRemainingMs(nowMs) > 0) {
    return false;
  }

  _activePattern = pattern;
  _activeIsAuto = isAuto;
  _burstStartMs = nowMs;
  _phaseStartMs = nowMs;

  if (pattern == LfWakePattern::CW) {
    uint32_t clamped = (durationMs == 0) ? LF_DEFAULT_BURST_MS : durationMs;
    if (clamped < 10U) {
      clamped = 10U;
    } else if (clamped > LF_MAX_BURST_MS) {
      clamped = LF_MAX_BURST_MS;
    }
    _targetDurationMs = clamped;
    _state = State::CW_ACTIVE;
    startCarrier();
  } else {
    // Multi-pulse preamble: 5x (15 ms ON / 10 ms OFF) + 100 ms CW tail
    _pulseIndex = 0;
    _targetDurationMs = (LF_PULSE_COUNT * (LF_PULSE_ON_MS + LF_PULSE_OFF_MS)) + LF_PULSE_TAIL_CW_MS;
    _state = State::PULSE_ON;
    startCarrier();
  }

  return true;
}

void LfWakeDriver::abort() {
  stopCarrier();
  if (_state != State::IDLE) {
    _state = State::IDLE;
    _lastFinishedMs = millis();
    _hasFinishedOnce = true;
  }
}

void LfWakeDriver::setAutoWake(bool enabled, uint32_t intervalMs) {
  _autoEnabled = enabled;
  if (intervalMs > 0) {
    _autoIntervalMs = (intervalMs < LF_MIN_AUTO_INTERVAL_MS) ? LF_MIN_AUTO_INTERVAL_MS : intervalMs;
  }
  if (enabled) {
    // Allow an initial burst promptly when enabling auto-wake
    _lastAutoTriggerMs = millis() - _autoIntervalMs;
  }
}

uint32_t LfWakeDriver::cooldownRemainingMs(uint32_t nowMs) const {
  if (!_hasFinishedOnce) {
    return 0;
  }
  const uint32_t elapsed = nowMs - _lastFinishedMs;
  if (elapsed >= LF_MIN_COOLDOWN_MS) {
    return 0;
  }
  return LF_MIN_COOLDOWN_MS - elapsed;
}

void LfWakeDriver::finishBurst(uint32_t nowMs) {
  stopCarrier();
  _state = State::IDLE;
  _lastFinishedMs = nowMs;
  _hasFinishedOnce = true;
  ++_burstCount;

  _completedEvent.pattern = _activePattern;
  _completedEvent.duration_ms = nowMs - _burstStartMs;
  _completedEvent.is_auto = _activeIsAuto;
  _completedEvent.timestamp_ms = nowMs;
  _hasCompletedEvent = true;
}

void LfWakeDriver::update(uint32_t nowMs) {
  switch (_state) {
    case State::IDLE:
      if (_autoEnabled && (nowMs - _lastAutoTriggerMs) >= _autoIntervalMs) {
        if (triggerBurst(LfWakePattern::PULSED, 0, true)) {
          _lastAutoTriggerMs = nowMs;
        }
      }
      break;

    case State::CW_ACTIVE:
      if ((nowMs - _phaseStartMs) >= _targetDurationMs) {
        finishBurst(nowMs);
      }
      break;

    case State::PULSE_ON:
      if ((nowMs - _phaseStartMs) >= LF_PULSE_ON_MS) {
        stopCarrier();
        _phaseStartMs = nowMs;
        _state = State::PULSE_OFF;
      }
      break;

    case State::PULSE_OFF:
      if ((nowMs - _phaseStartMs) >= LF_PULSE_OFF_MS) {
        ++_pulseIndex;
        _phaseStartMs = nowMs;
        if (_pulseIndex < LF_PULSE_COUNT) {
          _state = State::PULSE_ON;
          startCarrier();
        } else {
          _state = State::PULSE_TAIL_CW;
          startCarrier();
        }
      }
      break;

    case State::PULSE_TAIL_CW:
      if ((nowMs - _phaseStartMs) >= LF_PULSE_TAIL_CW_MS) {
        finishBurst(nowMs);
      }
      break;
  }
}

bool LfWakeDriver::popCompletedEvent(LfWakeEvent& out) {
  if (!_hasCompletedEvent) {
    return false;
  }
  out = _completedEvent;
  _hasCompletedEvent = false;
  return true;
}
