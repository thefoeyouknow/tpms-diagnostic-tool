// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_soc.h>
#include "ble_tpms.h"
#include "config.h"
#include "lf_wake.h"
#include "power_monitor.h"
#include "rf_receiver.h"
#include "telemetry_ble.h"
#include "tpms_decoder.h"

namespace {

BleTpmsReceiver    g_bleTpms;
PowerMonitor       g_power;
LfWakeDriver       g_lfWake;
RfReceiver         g_rf;
TpmsDecoder        g_decoder;
TelemetryInterface g_telemetry;

SemaphoreHandle_t s_wakeSem               = nullptr;
uint8_t           s_wakeBtnStable         = HIGH;
uint8_t           s_wakeBtnLastRaw        = HIGH;
uint32_t          s_wakeBtnChangeMs       = 0U;
bool              s_ignoreDfuUntilRelease = false;
uint32_t          s_lastActivityMs        = 0U;

void onWakeButtonIsr() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (s_wakeSem) {
    xSemaphoreGiveFromISR(s_wakeSem, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

void startBlueBreathPwm() {
  HwPWM0.addPin(PIN_BLE_LED);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);
  HwPWM0.setMaxValue(128);
  HwPWM0.writePin(PIN_BLE_LED, 0, false);
}

void stopBlueBreathPwm() {
  HwPWM0.writePin(PIN_BLE_LED, 0, false);
  HwPWM0.stop();
  HwPWM0.removePin(PIN_BLE_LED);
  pinMode(PIN_BLE_LED, OUTPUT);
  digitalWrite(PIN_BLE_LED, LOW);
}

[[noreturn]] void enterStage2PowerOff(bool alreadyPrepared) {
  stopBlueBreathPwm();
  detachInterrupt(PIN_WAKE_BUTTON);
  g_lfWake.abort();

  if (!alreadyPrepared) {
    g_bleTpms.setEnabled(false);
    g_rf.sleep();
    g_telemetry.prepareSleep(true);
  }

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  pinMode(PIN_BLE_LED, OUTPUT);
  digitalWrite(PIN_BLE_LED, LOW);
  pinMode(PIN_LF_WAKE_PWM, OUTPUT);
  digitalWrite(PIN_LF_WAKE_PWM, LOW);

  // Do not arm SENSE on P0.20 (DFU) because waking via DFU traps in bootloader DFU check;
  // wake from Stage 2 System OFF cleanly via the onboard RESET button (P0.21).
  pinMode(PIN_WAKE_BUTTON, INPUT);
  delay(30);

  (void)sd_power_system_off();
  NRF_POWER->SYSTEMOFF = 1;
  while (true) {
    __WFI();
  }
}

void enterStage1Sleep(uint32_t stage2WindowMs) {
  const bool bleScanWasEnabled = g_bleTpms.isEnabled();
  g_lfWake.abort();
  g_bleTpms.setEnabled(false);
  g_rf.sleep();
  g_telemetry.prepareSleep(false);

  if (!s_wakeSem) {
    s_wakeSem = xSemaphoreCreateBinary();
  }
  while (s_wakeSem && xSemaphoreTake(s_wakeSem, 0) == pdTRUE) {
  }

  pinMode(PIN_WAKE_BUTTON, INPUT_PULLUP);
  attachInterrupt(PIN_WAKE_BUTTON, onWakeButtonIsr, FALLING);

  const uint32_t sleepStartMs = millis();
  bool wokeByDfu = false;

  while (!wokeByDfu) {
    const uint32_t elapsed = millis() - sleepStartMs;
    if (elapsed >= stage2WindowMs) {
      break;
    }

    // ~1.2 s smooth blue breathing pulse (600 ms fade-in + 600 ms fade-out)
    startBlueBreathPwm();
    for (uint8_t step = 0U; step < 40U; ++step) {
      const uint32_t lvl = (step < 20U) ? static_cast<uint32_t>(step + 1U)
                                        : static_cast<uint32_t>(40U - step);
      const uint16_t duty = static_cast<uint16_t>((lvl * lvl * 96U) / 400U);
      HwPWM0.writePin(PIN_BLE_LED, duty, false);

      if (s_wakeSem && xSemaphoreTake(s_wakeSem, pdMS_TO_TICKS(30)) == pdTRUE) {
        wokeByDfu = true;
        break;
      }
      if ((millis() - sleepStartMs) >= stage2WindowMs) {
        break;
      }
    }
    stopBlueBreathPwm();

    if (wokeByDfu) {
      break;
    }

    const uint32_t afterBreathElapsed = millis() - sleepStartMs;
    if (afterBreathElapsed >= stage2WindowMs) {
      break;
    }

    // Dark low-power interval for remainder of 3.6 s cycle
    const uint32_t darkTargetMs = (SLEEP_BREATH_CYCLE_MS > 1200UL) ? (SLEEP_BREATH_CYCLE_MS - 1200UL) : 1000UL;
    const uint32_t remStage2Ms = stage2WindowMs - afterBreathElapsed;
    const uint32_t waitMs = (darkTargetMs < remStage2Ms) ? darkTargetMs : remStage2Ms;
    if (s_wakeSem && xSemaphoreTake(s_wakeSem, pdMS_TO_TICKS(waitMs)) == pdTRUE) {
      wokeByDfu = true;
      break;
    }
  }

  stopBlueBreathPwm();
  detachInterrupt(PIN_WAKE_BUTTON);

  if (!wokeByDfu) {
    enterStage2PowerOff(true);
  }

  // Wake-only DFU transition: suppress 125 kHz trigger & 5 Hz blink until DFU is released.
  s_ignoreDfuUntilRelease = true;
  s_wakeBtnStable = LOW;
  s_wakeBtnLastRaw = static_cast<uint8_t>(digitalRead(PIN_WAKE_BUTTON));
  s_wakeBtnChangeMs = millis();
  s_lastActivityMs = millis();

  (void)g_rf.wake();
  g_telemetry.resumeFromSleep(bleScanWasEnabled);
}

void initWakeButton() {
  pinMode(PIN_WAKE_BUTTON, INPUT_PULLUP);
  const uint8_t level = static_cast<uint8_t>(digitalRead(PIN_WAKE_BUTTON));
  s_wakeBtnStable = level;
  s_wakeBtnLastRaw = level;
  s_wakeBtnChangeMs = millis();
}

void pollWakeButton(uint32_t nowMs) {
  const uint8_t raw = static_cast<uint8_t>(digitalRead(PIN_WAKE_BUTTON));
  if (raw != s_wakeBtnLastRaw) {
    s_wakeBtnLastRaw = raw;
    s_wakeBtnChangeMs = nowMs;
  }

  const bool debounced = ((nowMs - s_wakeBtnChangeMs) >= WAKE_BUTTON_DEBOUNCE_MS);
  if (s_ignoreDfuUntilRelease) {
    if (debounced && raw == HIGH) {
      s_ignoreDfuUntilRelease = false;
      s_wakeBtnStable = HIGH;
    }
    g_telemetry.setStatusLedHold(false);
    return;
  }

  if (debounced && raw != s_wakeBtnStable) {
    s_wakeBtnStable = raw;
    if (s_wakeBtnStable == LOW) {
      s_lastActivityMs = nowMs;
      g_telemetry.setStatusLedHold(true);
      (void)g_lfWake.triggerBurst(LfWakePattern::PULSED, 0, false);
    } else {
      g_telemetry.setStatusLedHold(false);
    }
  }

  if (s_wakeBtnStable == LOW) {
    s_lastActivityMs = nowMs;
    g_telemetry.setStatusLedHold(true);
    if (!g_lfWake.isActive() && g_lfWake.cooldownRemainingMs(nowMs) == 0U) {
      (void)g_lfWake.triggerBurst(LfWakePattern::PULSED, 0, false);
    }
  } else {
    g_telemetry.setStatusLedHold(false);
  }
}

}  // namespace

void setup() {
  // Ground LF MOSFET gate first before any peripheral initialization
  g_lfWake.begin();
  initWakeButton();
  s_wakeSem = xSemaphoreCreateBinary();

  Serial.begin(TPMS_SERIAL_BAUD);
  delay(50);

  g_power.begin();
  g_decoder.reset();

  const bool radioOk = g_rf.begin();
  g_telemetry.begin(g_power, g_lfWake, g_rf, g_decoder, g_bleTpms);
  g_bleTpms.begin(true);

  if (!radioOk) {
    Serial.printf(
        "{\"type\":\"error\",\"msg\":\"cc1101 init failed\",\"err\":%d,\"ver\":\"0x%02X\"}\n",
        g_rf.lastError(),
        static_cast<unsigned int>(g_rf.chipVersion() & 0xFF));
  }

  s_lastActivityMs = millis();
  g_telemetry.emitStatus();
}

void loop() {
  const uint32_t nowMs = millis();

  pollWakeButton(nowMs);
  g_power.update(nowMs);
  g_lfWake.update(nowMs);

  LfWakeEvent wakeEvt{};
  if (g_lfWake.popCompletedEvent(wakeEvt)) {
    g_telemetry.emitLfWake(wakeEvt);
  }

  static bool s_inFrame = false;
  static float s_frameRssi = -128.0f;

  PulseEvent pulse{};
  uint16_t processed = 0;
  constexpr uint16_t kMaxPulsesPerLoop = 256U;
  while (processed < kMaxPulsesPerLoop && g_rf.popPulse(pulse)) {
    ++processed;
    TpmsPacket pkt{};
    if (g_decoder.feedPulse(
            pulse,
            g_rf.currentFreqMhz(),
            g_rf.currentMod(),
            s_frameRssi,
            pkt)) {
      if (pkt.rssi_dbm <= -127.0f) {
        pkt.rssi_dbm = g_rf.currentRssiDbm();
      }
      g_telemetry.triggerLedConfirmation();
      g_telemetry.emitTpmsPacket(pkt);
      s_inFrame = false;
      s_frameRssi = -128.0f;
    } else {
      const bool nowInFrame = g_decoder.isFrameInProgress();
      if (!s_inFrame && nowInFrame) {
        s_frameRssi = g_rf.currentRssiDbm();
      } else if (!nowInFrame) {
        s_frameRssi = -128.0f;
      }
      s_inFrame = nowInFrame;
    }
  }

  g_rf.update(nowMs, g_decoder.isFrameInProgress());

  TpmsPacket blePkt{};
  while (g_bleTpms.popPacket(blePkt)) {
    g_telemetry.triggerLedConfirmation();
    g_telemetry.emitTpmsPacket(blePkt);
  }

  BleRawAdvEvent rawBle{};
  while (g_bleTpms.popRawAdv(rawBle)) {
    g_telemetry.emitBleRawAdv(rawBle);
  }

  g_telemetry.update(nowMs);

  if (Bluefruit.connected() || g_telemetry.isUsbLikelyPresent(nowMs, false) || g_telemetry.takeActivityFlag()) {
    s_lastActivityMs = nowMs;
  }

  SleepAction pendingAction = SleepAction::NONE;
  if (g_telemetry.pollPendingSleep(nowMs, pendingAction)) {
    if (pendingAction == SleepAction::STAGE2_POWEROFF) {
      enterStage2PowerOff(false);
    } else if (pendingAction == SleepAction::STAGE1_SLEEP) {
      const uint32_t stage2WindowMs =
          (g_telemetry.poweroffTotalMs() > g_telemetry.sleepIdleMs())
              ? (g_telemetry.poweroffTotalMs() - g_telemetry.sleepIdleMs())
              : 0UL;
      enterStage1Sleep(stage2WindowMs);
      return;
    }
  }

  if (g_telemetry.autoSleepEnabled() &&
      static_cast<uint32_t>(nowMs - s_lastActivityMs) >= g_telemetry.sleepIdleMs()) {
    const uint32_t stage2WindowMs =
        (g_telemetry.poweroffTotalMs() > g_telemetry.sleepIdleMs())
            ? (g_telemetry.poweroffTotalMs() - g_telemetry.sleepIdleMs())
            : 0UL;
    enterStage1Sleep(stage2WindowMs);
    return;
  }

  if (processed < kMaxPulsesPerLoop) {
    delay(1);
  }
}
