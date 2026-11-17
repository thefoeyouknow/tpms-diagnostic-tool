#include <Arduino.h>
#include "config.h"
#include "lf_wake.h"
#include "power_monitor.h"
#include "rf_receiver.h"
#include "telemetry_ble.h"
#include "tpms_decoder.h"

namespace {

PowerMonitor       g_power;
LfWakeDriver       g_lfWake;
RfReceiver         g_rf;
TpmsDecoder        g_decoder;
TelemetryInterface g_telemetry;

}  // namespace

void setup() {
  // Ground LF MOSFET gate first before any peripheral initialization
  g_lfWake.begin();

  Serial.begin(TPMS_SERIAL_BAUD);
  delay(50);

  g_power.begin();
  g_decoder.reset();

  const bool radioOk = g_rf.begin();
  g_telemetry.begin(g_power, g_lfWake, g_rf, g_decoder);

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
  g_telemetry.update(nowMs);

  if (processed < kMaxPulsesPerLoop) {
    delay(1);
  }
}
