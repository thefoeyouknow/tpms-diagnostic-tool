// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>
#include "rf_receiver.h"

struct TpmsPacket {
  char     protocol[20];
  uint32_t sensor_id;
  float    freq_mhz;
  char     modulation[8];
  float    rssi_dbm;
  float    pressure_psi;
  float    pressure_kpa;
  float    temp_c;
  bool     battery_low;
  bool     motion;
  bool     fast_leak;
  bool     crc_ok;
  uint16_t bit_count;
  uint32_t bitrate_bps;
  uint8_t  raw_bytes[16];
  uint8_t  raw_len;
  uint32_t timestamp_ms;
};

class TpmsDecoder {
 public:
  void reset();
  void setRawMode(bool enabled) { _rawMode = enabled; }
  bool rawMode() const { return _rawMode; }

  // Feed a single GDO0 pulse into the Manchester slicer.
  // Returns true and populates `out` when a packet (or raw candidate if rawMode) is decoded.
  bool feedPulse(const PulseEvent& pulse,
                 float freqMhz,
                 RfModulation mod,
                 float rssiDbm,
                 TpmsPacket& out);

  // True while the decoder has locked onto a preamble/frame and is receiving bits.
  bool isFrameInProgress() const { return _state == State::IN_FRAME || _state == State::PREAMBLE_LOCK; }

  // Inject a synthetic Manchester-encoded TPMS test frame (0=Schrader, 1=Toyota, 2=Ford).
  bool injectTestPacket(uint8_t protoIndex,
                        float freqMhz,
                        RfModulation mod,
                        float rssiDbm,
                        TpmsPacket& out);

  uint32_t decodedCount() const { return _decodedCount; }
  uint32_t rawCandidateCount() const { return _rawCandidateCount; }

  static uint8_t crc8(const uint8_t* data, size_t len, uint8_t poly, uint8_t init);

 private:
  enum class State : uint8_t {
    SEARCH_PREAMBLE = 0,
    PREAMBLE_LOCK,
    IN_FRAME
  };

  static constexpr uint16_t kMaxFrameBits = 128U;
  static constexpr uint16_t kMinFrameBits = 48U;

  void resetToSearch();
  void pushChip(uint8_t chipLevel);
  bool finalizeFrame(float freqMhz, RfModulation mod, float rssiDbm, TpmsPacket& out);

  bool tryDecodeSchrader(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const;
  bool tryDecodeToyota(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const;
  bool tryDecodeFord(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const;
  bool tryDecodeContinental(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const;
  bool tryDecodeAllPolars(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const;

  static void packBits(const uint8_t* bits, uint16_t bitCount, uint8_t* bytesOut, uint8_t& byteLenOut);

  State    _state            = State::SEARCH_PREAMBLE;
  bool     _rawMode          = false;

  // Preamble timing estimator
  uint32_t _preambleSumUs    = 0;
  uint8_t  _preambleCount    = 0;
  uint16_t _halfBitUs        = 26;
  uint8_t  _lastLevel        = 0;

  // Manchester chip accumulator (2 chips = 1 data bit)
  uint8_t  _pendingChip      = 0xFF;
  uint8_t  _violationCount   = 0;
  bool     _sawSyncReset     = false;

  uint8_t  _bits[kMaxFrameBits]{};
  uint16_t _bitCount         = 0;

  uint32_t _decodedCount     = 0;
  uint32_t _rawCandidateCount = 0;
};
