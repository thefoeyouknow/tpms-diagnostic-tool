// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>

enum class RfBandMode : uint8_t {
  HOP = 0,
  BAND_315,
  BAND_433
};

enum class RfModMode : uint8_t {
  BOTH = 0,
  FSK_ONLY,
  OOK_ONLY
};

enum class RfModulation : uint8_t {
  FSK = 0,
  OOK
};

struct PulseEvent {
  uint16_t duration_us;
  uint8_t  level;      // Logic level of the pulse that just completed (0 or 1)
  uint8_t  reserved;
};

struct RfHardwareDiag {
  uint8_t  miso_pullup_cs_high;
  uint8_t  miso_pulldown_cs_high;
  uint8_t  miso_cs_low;
  uint8_t  gdo0_pu;
  uint8_t  gdo0_pd;
  bool     chip_rdy_seen;
  uint32_t gdo0_edges_2ms;
  uint8_t  bb_status;
  uint8_t  bb_partnum;
  uint8_t  bb_version;
  int16_t  hw_version;
  int8_t   clk_pin;
  uint32_t clk_edges;
  int8_t   rdy_csn;
  int8_t   rdy_miso;
};

class RfReceiver {
 public:
  bool begin();
  void sleep();
  bool wake();
  void update(uint32_t nowMs, bool activeBurstInProgress);

  bool popPulse(PulseEvent& out);
  void clearPulses();

  void setBandMode(RfBandMode mode);
  RfBandMode bandMode() const { return _bandMode; }

  void setModMode(RfModMode mode);
  RfModMode modMode() const { return _modMode; }

  float currentFreqMhz() const { return _currentFreqMhz; }
  RfModulation currentMod() const { return _currentMod; }
  float currentRssiDbm();

  bool isRadioOk() const { return _radioOk; }
  int16_t chipVersion() const { return _chipVersion; }
  int16_t lastError() const { return _lastError; }

  uint32_t totalPulses() const;
  uint32_t droppedPulses() const;

  void sampleBandRssi(float& rssi315Dbm, float& rssi433Dbm);
  RfHardwareDiag runHardwareDiag();
  const RfHardwareDiag& lastHardwareDiag() const { return _hwDiag; }
  void setBenchPinLevels(bool high, uint8_t& misoOut, uint8_t& gdo0Out);

 private:
  bool beginInternal(bool runPinMatrixScan);
  void probeAndResetCc1101(bool runPinMatrixScan);
  bool applyChannel(float freqMhz, RfModulation mod, bool forceConfig = false);
  void initTimer1Micros();
  static void onGdo0Edge();

  RfBandMode     _bandMode       = RfBandMode::HOP;
  RfModMode      _modMode        = RfModMode::BOTH;
  float          _currentFreqMhz = 315.00f;
  RfModulation   _currentMod     = RfModulation::FSK;
  uint8_t        _hopStep        = 0;

  uint32_t       _lastHopMs      = 0;
  uint32_t       _deferStartMs   = 0;
  bool           _deferringHop   = false;

  bool           _radioOk        = false;
  int16_t        _chipVersion    = -1;
  int16_t        _lastError      = 0;
  RfHardwareDiag _hwDiag{};
};
