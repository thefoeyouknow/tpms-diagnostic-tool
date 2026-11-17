// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include <Arduino.h>
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

uint8_t  s_wakeBtnStable    = HIGH;
uint8_t  s_wakeBtnLastRaw   = HIGH;
uint32_t s_wakeBtnChangeMs  = 0U;

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

  if ((nowMs - s_wakeBtnChangeMs) < WAKE_BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (raw != s_wakeBtnStable) {
    s_wakeBtnStable = raw;
    if (s_wakeBtnStable == LOW) {
      if (g_lfWake.triggerBurst(LfWakePattern::PULSED, 0, false)) {
        g_telemetry.triggerLedConfirmation();
      }
    }
  }
}

}  // namespace

void setup() {
  // Ground LF MOSFET gate first before any peripheral initialization
  g_lfWake.begin();
  initWakeButton();

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

  if (processed < kMaxPulsesPerLoop) {
    delay(1);
  }
}
