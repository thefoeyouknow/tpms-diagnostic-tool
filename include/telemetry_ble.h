// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>
#include "ble_tpms.h"
#include "power_monitor.h"
#include "lf_wake.h"
#include "rf_receiver.h"
#include "tpms_decoder.h"

enum class SleepAction : uint8_t {
  NONE = 0,
  STAGE1_SLEEP,
  STAGE2_POWEROFF
};

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
  void emitPowerState(const char* state, const char* target, uint32_t delayMs = 0U, const char* src = nullptr);

  void triggerLedConfirmation();
  void setStatusLedHold(bool holdActive);

  // Power & sleep coordination
  bool isUsbLikelyPresent(uint32_t nowMs, bool forceProbe = false);
  bool usbRxPulldownHigh() const { return _usbRxPd; }
  bool autoSleepEnabled() const { return _autoSleepEnabled; }
  uint32_t sleepIdleMs() const { return _sleepIdleMs; }
  uint32_t poweroffTotalMs() const { return _poweroffTotalMs; }
  bool takeActivityFlag();
  bool pollPendingSleep(uint32_t nowMs, SleepAction& outAction);
  void prepareSleep(bool finalPowerOff = false);
  void resumeFromSleep(bool bleScanWasEnabled);

 private:
  void pollStreamCommands(Stream& stream, char* buf, size_t& len, size_t cap, bool isSerial);
  void handleCommand(const char* rawCmd);
  void emitLine(const char* line);
  void updateLed(uint32_t nowMs);
  void updateBatteryService();
  bool probeUsbRxPulldown();
  void scheduleSleep(SleepAction action, uint32_t delayMs, const char* src);

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

  bool     _ledOn               = false;
  uint32_t _ledOffAtMs          = 0;
  bool     _ledHoldActive       = false;
  bool     _ledHoldPhase        = false;
  uint32_t _ledHoldNextToggleMs = 0;

  uint32_t _lastHeartbeatMs = 0;
  uint8_t  _lastBleSoc      = 255U;

  bool     _hasSerialRx         = false;
  uint32_t _lastSerialRxMs      = 0;
  bool     _usbProbeInit        = false;
  bool     _usbRxPd             = false;
  uint32_t _lastUsbProbeMs      = 0;
  bool     _activityFlag        = false;

  bool        _autoSleepEnabled   = true;
  uint32_t    _sleepIdleMs        = DEFAULT_SLEEP_IDLE_MS;
  uint32_t    _poweroffTotalMs    = DEFAULT_POWEROFF_TOTAL_MS;
  SleepAction _pendingSleepAction = SleepAction::NONE;
  uint32_t    _pendingSleepAtMs   = 0;
};
