// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>
#include "ble_tpms.h"
#include "power_monitor.h"
#include "lf_wake.h"
#include "rf_receiver.h"
#include "tpms_decoder.h"

class TelemetryInterface {
 public:
  void begin(PowerMonitor& power,
             LfWakeDriver& lfWake,
             RfReceiver& rf,
             TpmsDecoder& decoder,
             BleTpmsReceiver& bleTpms);

  void update(uint32_t nowMs);

  void emitTpmsPacket(const TpmsPacket& pkt);
  void emitBleRawAdv(const BleRawAdvEvent& adv);
  void emitStatus();
  void emitLfWake(const LfWakeEvent& evt);
  void emitDiagnostics();
  void emitHelp();
  void triggerLedConfirmation();

 private:
  void pollStreamCommands(Stream& stream, char* buf, size_t& len, size_t cap);
  void handleCommand(const char* rawCmd);
  void emitLine(const char* line);
  void updateLed(uint32_t nowMs);
  void updateBatteryService();

  static const char* bandModeStr(RfBandMode mode);
  static const char* modModeStr(RfModMode mode);
  static const char* modStr(RfModulation mod);

  PowerMonitor*    _power   = nullptr;
  LfWakeDriver*    _lfWake  = nullptr;
  RfReceiver*      _rf      = nullptr;
  TpmsDecoder*     _decoder = nullptr;
  BleTpmsReceiver* _bleTpms = nullptr;

  char   _serialBuf[96]{};
  size_t _serialLen = 0;
  char   _bleBuf[96]{};
  size_t _bleLen    = 0;

  bool     _ledOn          = false;
  uint32_t _ledOffAtMs     = 0;
  uint32_t _lastHeartbeatMs = 0;
  uint8_t  _lastBleSoc     = 255U;
};
