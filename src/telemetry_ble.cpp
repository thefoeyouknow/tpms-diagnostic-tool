// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include "telemetry_ble.h"
#include "config.h"
#include <bluefruit.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

BLEDis  g_bledis;
BLEUart g_bleuart;
BLEBas  g_blebas;

volatile bool     s_bleJustSubscribed = false;
volatile uint32_t s_bleSubscribedAtMs = 0;

void onBleUartNotify(uint16_t /*conn_hdl*/, bool enabled) {
  if (enabled) {
    s_bleSubscribedAtMs = millis();
    s_bleJustSubscribed = true;
  }
}

void formatHex(const uint8_t* bytes, uint8_t len, char* out, size_t outCap) {
  size_t pos = 0;
  for (uint8_t i = 0; i < len && (pos + 2U) < outCap; ++i) {
    snprintf(out + pos, outCap - pos, "%02X", bytes[i]);
    pos += 2U;
  }
  out[pos] = '\0';
}

void toUpperTrimmed(const char* src, char* dst, size_t dstCap) {
  while (*src && isspace(static_cast<unsigned char>(*src))) {
    ++src;
  }
  size_t n = 0;
  while (*src && (n + 1U) < dstCap) {
    dst[n++] = static_cast<char>(toupper(static_cast<unsigned char>(*src++)));
  }
  while (n > 0U && isspace(static_cast<unsigned char>(dst[n - 1U]))) {
    --n;
  }
  dst[n] = '\0';
}

}  // namespace

void TelemetryInterface::begin(PowerMonitor& power,
                               LfWakeDriver& lfWake,
                               RfReceiver& rf,
                               TpmsDecoder& decoder,
                               BleTpmsReceiver& bleTpms) {
  _power = &power;
  _lfWake = &lfWake;
  _rf = &rf;
  _decoder = &decoder;
  _bleTpms = &bleTpms;

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  _ledOn = false;

  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(TPMS_BLE_DEVICE_NAME);

  g_bledis.setManufacturer("Bench-Inventory");
  g_bledis.setModel("nRF52-CC1101-TPMS");
  g_bledis.setFirmwareRev(TPMS_FW_VERSION);
  g_bledis.begin();

  g_bleuart.setNotifyCallback(onBleUartNotify);
  g_bleuart.begin();

  const PowerStatus pwr = _power->status();
  g_blebas.begin();
  g_blebas.write(pwr.soc_percent);
  _lastBleSoc = pwr.soc_percent;

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(g_bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  _lastHeartbeatMs = millis();
}

void TelemetryInterface::update(uint32_t nowMs) {
  pollStreamCommands(Serial, _serialBuf, _serialLen, sizeof(_serialBuf));
  pollStreamCommands(g_bleuart, _bleBuf, _bleLen, sizeof(_bleBuf));

  updateLed(nowMs);
  updateBatteryService();

  if (s_bleJustSubscribed && (nowMs - s_bleSubscribedAtMs) >= 50U) {
    s_bleJustSubscribed = false;
    _lastHeartbeatMs = nowMs;
    emitStatus();
  } else if ((nowMs - _lastHeartbeatMs) >= STATUS_HEARTBEAT_MS) {
    _lastHeartbeatMs = nowMs;
    emitStatus();
  }
}

void TelemetryInterface::pollStreamCommands(Stream& stream, char* buf, size_t& len, size_t cap) {
  while (stream.available() > 0) {
    const int c = stream.read();
    if (c < 0) {
      break;
    }
    if (c == '\r' || c == '\n') {
      if (len > 0U) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0U;
      }
    } else if (len + 1U < cap) {
      buf[len++] = static_cast<char>(c);
    } else {
      // Overflow guard: discard oversized line and reset
      len = 0U;
    }
  }
}

void TelemetryInterface::handleCommand(const char* rawCmd) {
  char cmd[96]{};
  toUpperTrimmed(rawCmd, cmd, sizeof(cmd));
  if (cmd[0] == '\0') {
    return;
  }

  if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
    emitHelp();
    return;
  }

  if (strcmp(cmd, "STATUS") == 0) {
    emitStatus();
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    const PowerStatus pwr = _power->status();
    char pong[160];
    snprintf(
        pong, sizeof(pong),
        "{\"type\":\"pong\",\"freq_mhz\":%.2f,\"rssi_dbm\":%.1f,\"vbat_v\":%.2f,\"ts_ms\":%lu}",
        static_cast<double>(_rf->currentFreqMhz()),
        static_cast<double>(_rf->currentRssiDbm()),
        static_cast<double>(pwr.vbat_v),
        static_cast<unsigned long>(millis()));
    emitLine(pong);
    return;
  }

  if (strcmp(cmd, "DIAG") == 0) {
    emitDiagnostics();
    return;
  }

  if (strcmp(cmd, "WAKE") == 0) {
    if (!_lfWake->triggerBurst(LfWakePattern::CW, 0, false)) {
      char err[96];
      snprintf(err, sizeof(err),
               "{\"type\":\"error\",\"msg\":\"lf busy or cooldown\",\"cooldown_ms\":%lu}",
               static_cast<unsigned long>(_lfWake->cooldownRemainingMs(millis())));
      emitLine(err);
    }
    return;
  }

  if (strncmp(cmd, "WAKE CW", 7) == 0) {
    uint32_t dur = 0;
    if (cmd[7] != '\0') {
      dur = static_cast<uint32_t>(strtoul(cmd + 7, nullptr, 10));
    }
    if (!_lfWake->triggerBurst(LfWakePattern::CW, dur, false)) {
      char err[96];
      snprintf(err, sizeof(err),
               "{\"type\":\"error\",\"msg\":\"lf busy or cooldown\",\"cooldown_ms\":%lu}",
               static_cast<unsigned long>(_lfWake->cooldownRemainingMs(millis())));
      emitLine(err);
    }
    return;
  }

  if (strcmp(cmd, "WAKE PULSE") == 0) {
    if (!_lfWake->triggerBurst(LfWakePattern::PULSED, 0, false)) {
      char err[96];
      snprintf(err, sizeof(err),
               "{\"type\":\"error\",\"msg\":\"lf busy or cooldown\",\"cooldown_ms\":%lu}",
               static_cast<unsigned long>(_lfWake->cooldownRemainingMs(millis())));
      emitLine(err);
    }
    return;
  }

  if (strncmp(cmd, "WAKE AUTO ON", 12) == 0) {
    uint32_t interval = 0;
    if (cmd[12] != '\0') {
      interval = static_cast<uint32_t>(strtoul(cmd + 12, nullptr, 10));
    }
    _lfWake->setAutoWake(true, interval);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "WAKE AUTO OFF") == 0) {
    _lfWake->setAutoWake(false, 0);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BAND 315") == 0) {
    _rf->setBandMode(RfBandMode::BAND_315);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BAND 433") == 0) {
    _rf->setBandMode(RfBandMode::BAND_433);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BAND HOP") == 0) {
    _rf->setBandMode(RfBandMode::HOP);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "MOD FSK") == 0) {
    _rf->setModMode(RfModMode::FSK_ONLY);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "MOD OOK") == 0) {
    _rf->setModMode(RfModMode::OOK_ONLY);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "MOD BOTH") == 0) {
    _rf->setModMode(RfModMode::BOTH);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "RAW ON") == 0) {
    _decoder->setRawMode(true);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "RAW OFF") == 0) {
    _decoder->setRawMode(false);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BLETPMS ON") == 0 || strcmp(cmd, "BLESCAN ON") == 0) {
    if (_bleTpms) _bleTpms->setEnabled(true);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BLETPMS OFF") == 0 || strcmp(cmd, "BLESCAN OFF") == 0) {
    if (_bleTpms) _bleTpms->setEnabled(false);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BLERAW ON") == 0) {
    if (_bleTpms) _bleTpms->setRawMode(true);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "BLERAW OFF") == 0) {
    if (_bleTpms) _bleTpms->setRawMode(false);
    emitStatus();
    return;
  }

  if (strcmp(cmd, "PINLOW") == 0) {
    uint8_t miso = 0, gdo0 = 0;
    _rf->setBenchPinLevels(false, miso, gdo0);
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"type\":\"pinlow\",\"miso\":%u,\"gdo0\":%u}",
             static_cast<unsigned int>(miso), static_cast<unsigned int>(gdo0));
    emitLine(msg);
    return;
  }

  if (strcmp(cmd, "PINHIGH") == 0) {
    uint8_t miso = 0, gdo0 = 0;
    _rf->setBenchPinLevels(true, miso, gdo0);
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"type\":\"pinhigh\",\"miso\":%u,\"gdo0\":%u}",
             static_cast<unsigned int>(miso), static_cast<unsigned int>(gdo0));
    emitLine(msg);
    return;
  }

  if (strcmp(cmd, "TESTPKT") == 0 || strncmp(cmd, "TESTPKT ", 8) == 0) {
    uint8_t protoIdx = 0U;
    bool testBle = false;
    bool validArg = true;
    if (strncmp(cmd, "TESTPKT ", 8) == 0) {
      const char* arg = cmd + 8;
      while (*arg == ' ') {
        ++arg;
      }
      if (strcmp(arg, "SCHRADER") == 0) {
        protoIdx = 0U;
      } else if (strcmp(arg, "TOYOTA") == 0) {
        protoIdx = 1U;
      } else if (strcmp(arg, "FORD") == 0) {
        protoIdx = 2U;
      } else if (strcmp(arg, "BLE") == 0) {
        testBle = true;
      } else {
        validArg = false;
      }
    }
    if (!validArg) {
      emitLine("{\"type\":\"error\",\"msg\":\"usage: TESTPKT [SCHRADER|TOYOTA|FORD|BLE]\"}");
      return;
    }

    TpmsPacket pkt{};
    const bool decoded = testBle
        ? (_bleTpms && _bleTpms->injectTestPacket(pkt))
        : _decoder->injectTestPacket(protoIdx, _rf->currentFreqMhz(), _rf->currentMod(), -52.0f, pkt);
    if (decoded) {
      triggerLedConfirmation();
      emitTpmsPacket(pkt);
    } else {
      emitLine("{\"type\":\"error\",\"msg\":\"testpkt decode failed\"}");
    }
    return;
  }

  char err[144];
  snprintf(err, sizeof(err), "{\"type\":\"error\",\"msg\":\"unknown command\",\"cmd\":\"%s\"}", cmd);
  emitLine(err);
}

void TelemetryInterface::emitLine(const char* line) {
  Serial.println(line);

  if (Bluefruit.connected() && g_bleuart.notifyEnabled()) {
    const uint16_t connHdl = Bluefruit.connHandle();
    BLEConnection* conn = Bluefruit.Connection(connHdl);
    uint16_t chunkMax = 20U;
    if (conn && conn->getMtu() > 3U) {
      chunkMax = static_cast<uint16_t>(conn->getMtu() - 3U);
      if (chunkMax > 200U) {
        chunkMax = 200U;
      }
    }

    const size_t len = strlen(line);
    size_t offset = 0U;
    while (offset < len) {
      const size_t chunk = ((len - offset) > chunkMax) ? chunkMax : (len - offset);
      size_t sent = 0U;
      for (uint8_t attempt = 0; attempt < 4U && sent == 0U; ++attempt) {
        if (!Bluefruit.connected() || !g_bleuart.notifyEnabled(connHdl)) {
          return;
        }
        sent = g_bleuart.write(connHdl, reinterpret_cast<const uint8_t*>(line + offset), chunk);
        if (sent == 0U) {
          delay(4);
        }
      }
      if (sent == 0U) {
        return;
      }
      offset += sent;
    }
    const uint8_t nl = '\n';
    for (uint8_t attempt = 0; attempt < 4U; ++attempt) {
      if (!Bluefruit.connected() || !g_bleuart.notifyEnabled(connHdl)) {
        return;
      }
      if (g_bleuart.write(connHdl, &nl, 1U) > 0U) {
        break;
      }
      delay(4);
    }
  }
}

void TelemetryInterface::emitTpmsPacket(const TpmsPacket& pkt) {
  char rawHex[34]{};
  formatHex(pkt.raw_bytes, pkt.raw_len, rawHex, sizeof(rawHex));

  char json[352];
  snprintf(
      json, sizeof(json),
      "{\"type\":\"tpms\",\"proto\":\"%s\",\"id\":\"0x%08lX\",\"freq_mhz\":%.2f,"
      "\"mod\":\"%s\",\"rssi_dbm\":%.1f,\"pressure_psi\":%.1f,\"pressure_kpa\":%.1f,"
      "\"temp_c\":%.1f,\"bat_low\":%s,\"motion\":%s,\"fast_leak\":%s,\"crc_ok\":%s,"
      "\"bits\":%u,\"raw\":\"%s\",\"ts_ms\":%lu}",
      pkt.protocol,
      static_cast<unsigned long>(pkt.sensor_id),
      static_cast<double>(pkt.freq_mhz),
      pkt.modulation,
      static_cast<double>(pkt.rssi_dbm),
      static_cast<double>(pkt.pressure_psi),
      static_cast<double>(pkt.pressure_kpa),
      static_cast<double>(pkt.temp_c),
      pkt.battery_low ? "true" : "false",
      pkt.motion ? "true" : "false",
      pkt.fast_leak ? "true" : "false",
      pkt.crc_ok ? "true" : "false",
      static_cast<unsigned int>(pkt.bit_count),
      rawHex,
      static_cast<unsigned long>(pkt.timestamp_ms));
  emitLine(json);
}

void TelemetryInterface::emitBleRawAdv(const BleRawAdvEvent& adv) {
  char mac[18]{};
  snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
           adv.addr[5], adv.addr[4], adv.addr[3], adv.addr[2], adv.addr[1], adv.addr[0]);
  char msdHex[44]{};
  formatHex(adv.msd, adv.msd_len, msdHex, sizeof(msdHex));

  char json[200];
  snprintf(
      json, sizeof(json),
      "{\"type\":\"ble_adv\",\"mac\":\"%s\",\"name\":\"%s\",\"rssi_dbm\":%d,\"msd\":\"%s\",\"ts_ms\":%lu}",
      mac, adv.name, static_cast<int>(adv.rssi_dbm), msdHex, static_cast<unsigned long>(adv.timestamp_ms));
  emitLine(json);
}

void TelemetryInterface::emitStatus() {
  const PowerStatus pwr = _power->status();
  const float rssi = _rf->currentRssiDbm();
  const uint32_t nowMs = millis();

  char json[448];
  snprintf(
      json, sizeof(json),
      "{\"type\":\"status\",\"freq_mhz\":%.2f,\"band_mode\":\"%s\",\"mod\":\"%s\","
      "\"mod_mode\":\"%s\",\"rssi_dbm\":%.1f,\"radio_ok\":%s,\"vbat_v\":%.2f,"
      "\"vbat_mv\":%u,\"soc_pct\":%u,\"low_bat\":%s,\"ble_conn\":%s,\"raw_mode\":%s,"
      "\"ble_scan\":%s,\"ble_scan_ok\":%s,\"ble_raw\":%s,\"ble_pkts\":%lu,"
      "\"lf_active\":%s,\"lf_auto\":%s,\"lf_bursts\":%lu,\"pulses\":%lu,"
      "\"dropped\":%lu,\"pkts\":%lu,\"ts_ms\":%lu}",
      static_cast<double>(_rf->currentFreqMhz()),
      bandModeStr(_rf->bandMode()),
      modStr(_rf->currentMod()),
      modModeStr(_rf->modMode()),
      static_cast<double>(rssi),
      _rf->isRadioOk() ? "true" : "false",
      static_cast<double>(pwr.vbat_v),
      static_cast<unsigned int>(pwr.vbat_mv),
      static_cast<unsigned int>(pwr.soc_percent),
      pwr.low_battery ? "true" : "false",
      Bluefruit.connected() ? "true" : "false",
      _decoder->rawMode() ? "true" : "false",
      (_bleTpms && _bleTpms->isEnabled()) ? "true" : "false",
      (_bleTpms && _bleTpms->isScannerRunning()) ? "true" : "false",
      (_bleTpms && _bleTpms->rawMode()) ? "true" : "false",
      static_cast<unsigned long>(_bleTpms ? _bleTpms->decodedCount() : 0UL),
      _lfWake->isActive() ? "true" : "false",
      _lfWake->isAutoWake() ? "true" : "false",
      static_cast<unsigned long>(_lfWake->burstCount()),
      static_cast<unsigned long>(_rf->totalPulses()),
      static_cast<unsigned long>(_rf->droppedPulses()),
      static_cast<unsigned long>(_decoder->decodedCount() + (_bleTpms ? _bleTpms->decodedCount() : 0UL)),
      static_cast<unsigned long>(nowMs));
  emitLine(json);
}

void TelemetryInterface::emitLfWake(const LfWakeEvent& evt) {
  const PowerStatus pwr = _power->sampleNow();
  const char* pat = (evt.pattern == LfWakePattern::PULSED) ? "PULSED" : "CW";

  char json[200];
  snprintf(
      json, sizeof(json),
      "{\"type\":\"lf_wake\",\"pattern\":\"%s\",\"duration_ms\":%lu,\"freq_hz\":%lu,"
      "\"duty_pct\":50,\"auto\":%s,\"vbat_v\":%.2f,\"ts_ms\":%lu}",
      pat,
      static_cast<unsigned long>(evt.duration_ms),
      static_cast<unsigned long>(LF_PWM_FREQ_HZ),
      evt.is_auto ? "true" : "false",
      static_cast<double>(pwr.vbat_v),
      static_cast<unsigned long>(evt.timestamp_ms));
  emitLine(json);
}

void TelemetryInterface::emitDiagnostics() {
  const PowerStatus pwr = _power->sampleNow();
  const RfHardwareDiag hw = _rf->runHardwareDiag();
  float rssi315 = -128.0f;
  float rssi433 = -128.0f;
  _rf->sampleBandRssi(rssi315, rssi433);

  char json[560];
  snprintf(
      json, sizeof(json),
      "{\"type\":\"diag\",\"radio_ok\":%s,\"err\":%d,\"cc1101_ver\":\"0x%02X\","
      "\"miso_hi\":%u,\"miso_lo\":%u,\"miso_csn_lo\":%u,\"gdo0_pu\":%u,\"gdo0_pd\":%u,\"chip_rdy\":%s,"
      "\"gdo0_edges_2ms\":%lu,\"clk_pin\":%d,\"clk_edges\":%lu,"
      "\"rdy_csn\":%d,\"rdy_miso\":%d,\"bb_status\":\"0x%02X\",\"bb_part\":\"0x%02X\","
      "\"bb_ver\":\"0x%02X\",\"rssi_315_dbm\":%.1f,\"rssi_433_dbm\":%.1f,"
      "\"vbat_v\":%.2f,\"vbat_mv\":%u,\"soc_pct\":%u,\"lf_pwm_hz\":%lu,"
      "\"lf_pin\":%u,\"pulses\":%lu,\"dropped\":%lu,\"ble_conn\":%s,\"ts_ms\":%lu}",
      _rf->isRadioOk() ? "true" : "false",
      static_cast<int>(_rf->lastError()),
      static_cast<unsigned int>(hw.hw_version & 0xFF),
      static_cast<unsigned int>(hw.miso_pullup_cs_high),
      static_cast<unsigned int>(hw.miso_pulldown_cs_high),
      static_cast<unsigned int>(hw.miso_cs_low),
      static_cast<unsigned int>(hw.gdo0_pu),
      static_cast<unsigned int>(hw.gdo0_pd),
      hw.chip_rdy_seen ? "true" : "false",
      static_cast<unsigned long>(hw.gdo0_edges_2ms),
      static_cast<int>(hw.clk_pin),
      static_cast<unsigned long>(hw.clk_edges),
      static_cast<int>(hw.rdy_csn),
      static_cast<int>(hw.rdy_miso),
      static_cast<unsigned int>(hw.bb_status),
      static_cast<unsigned int>(hw.bb_partnum),
      static_cast<unsigned int>(hw.bb_version),
      static_cast<double>(rssi315),
      static_cast<double>(rssi433),
      static_cast<double>(pwr.vbat_v),
      static_cast<unsigned int>(pwr.vbat_mv),
      static_cast<unsigned int>(pwr.soc_percent),
      static_cast<unsigned long>(LF_PWM_FREQ_HZ),
      static_cast<unsigned int>(PIN_LF_WAKE_PWM),
      static_cast<unsigned long>(_rf->totalPulses()),
      static_cast<unsigned long>(_rf->droppedPulses()),
      Bluefruit.connected() ? "true" : "false",
      static_cast<unsigned long>(millis()));
  emitLine(json);
}

void TelemetryInterface::emitHelp() {
  emitLine(
      "{\"type\":\"help\",\"cmds\":[\"HELP\",\"PING\",\"STATUS\",\"DIAG\",\"PINLOW\",\"PINHIGH\",\"WAKE\",\"WAKE CW [ms]\","
      "\"WAKE PULSE\",\"WAKE AUTO ON [ms]\",\"WAKE AUTO OFF\",\"BAND 315\",\"BAND 433\","
      "\"BAND HOP\",\"MOD FSK\",\"MOD OOK\",\"MOD BOTH\",\"RAW ON\",\"RAW OFF\",\"BLETPMS ON\",\"BLETPMS OFF\",\"BLERAW ON\",\"BLERAW OFF\",\"TESTPKT [SCHRADER|TOYOTA|FORD|BLE]\"]}");
}

void TelemetryInterface::triggerLedConfirmation() {
  digitalWrite(PIN_STATUS_LED, HIGH);
  _ledOn = true;
  _ledOffAtMs = millis() + LED_PULSE_MS;
}

void TelemetryInterface::updateLed(uint32_t nowMs) {
  if (_ledOn && static_cast<int32_t>(nowMs - _ledOffAtMs) >= 0) {
    digitalWrite(PIN_STATUS_LED, LOW);
    _ledOn = false;
  }
}

void TelemetryInterface::updateBatteryService() {
  const uint8_t soc = _power->status().soc_percent;
  if (soc != _lastBleSoc) {
    _lastBleSoc = soc;
    g_blebas.write(soc);
    if (Bluefruit.connected()) {
      g_blebas.notify(soc);
    }
  }
}

const char* TelemetryInterface::bandModeStr(RfBandMode mode) {
  switch (mode) {
    case RfBandMode::BAND_315: return "315";
    case RfBandMode::BAND_433: return "433";
    case RfBandMode::HOP:
    default: return "HOP";
  }
}

const char* TelemetryInterface::modModeStr(RfModMode mode) {
  switch (mode) {
    case RfModMode::FSK_ONLY: return "FSK";
    case RfModMode::OOK_ONLY: return "OOK";
    case RfModMode::BOTH:
    default: return "BOTH";
  }
}

const char* TelemetryInterface::modStr(RfModulation mod) {
  return (mod == RfModulation::OOK) ? "OOK" : "FSK";
}
