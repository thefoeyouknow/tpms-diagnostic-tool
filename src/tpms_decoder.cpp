// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include "tpms_decoder.h"
#include <string.h>

void TpmsDecoder::reset() {
  resetToSearch();
  _decodedCount = 0;
  _rawCandidateCount = 0;
}

void TpmsDecoder::resetToSearch() {
  _state = State::SEARCH_PREAMBLE;
  _preambleSumUs = 0;
  _preambleCount = 0;
  _halfBitUs = 26;
  _chipCount = 0;
}

uint8_t TpmsDecoder::crc8(const uint8_t* data, size_t len, uint8_t poly, uint8_t init) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80U) {
        crc = static_cast<uint8_t>((crc << 1) ^ poly);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

void TpmsDecoder::packBits(const uint8_t* bits, uint16_t bitCount, uint8_t* bytesOut, uint8_t& byteLenOut) {
  const uint8_t byteLen = static_cast<uint8_t>((bitCount + 7U) / 8U);
  memset(bytesOut, 0, 16);
  for (uint16_t i = 0; i < bitCount && i < 128U; ++i) {
    if (bits[i]) {
      bytesOut[i / 8U] |= static_cast<uint8_t>(1U << (7U - (i % 8U)));
    }
  }
  byteLenOut = (byteLen > 16U) ? 16U : byteLen;
}

void TpmsDecoder::pushRawChip(uint8_t chipLevel) {
  if (_chipCount < kMaxFrameChips) {
    _chips[_chipCount++] = chipLevel & 0x01U;
  }
}

uint16_t TpmsDecoder::extractManchester(const uint8_t* chips,
                                        uint16_t chipCount,
                                        uint16_t startChip,
                                        uint8_t* bitsOut,
                                        uint16_t maxBits) {
  uint16_t bitCount = 0U;
  uint16_t pos = startChip;
  while ((pos + 1U) < chipCount && bitCount < maxBits) {
    const uint8_t c0 = chips[pos];
    const uint8_t c1 = chips[pos + 1U];
    if (c0 == c1) {
      break;
    }
    bitsOut[bitCount++] = (c0 == 1U) ? 1U : 0U;
    pos += 2U;
  }
  return bitCount;
}

uint16_t TpmsDecoder::extractDiffManchester(const uint8_t* chips,
                                            uint16_t chipCount,
                                            uint16_t startChip,
                                            uint8_t* bitsOut,
                                            uint16_t maxBits) {
  if (startChip == 0U) {
    return 0U;
  }
  uint16_t bitCount = 0U;
  uint16_t pos = startChip;
  while ((pos + 1U) < chipCount && bitCount < maxBits) {
    const uint8_t prev = chips[pos - 1U];
    const uint8_t c0 = chips[pos];
    const uint8_t c1 = chips[pos + 1U];
    if (c0 == prev) {
      break;
    }
    bitsOut[bitCount++] = (c0 == c1) ? 1U : 0U;
    pos += 2U;
  }
  return bitCount;
}

bool TpmsDecoder::feedPulse(const PulseEvent& pulse,
                            float freqMhz,
                            RfModulation mod,
                            float rssiDbm,
                            TpmsPacket& out) {
  const uint16_t d = pulse.duration_us;
  const uint8_t lvl = pulse.level & 0x01U;

  // Reject sub-12 us RF glitches
  if (d < 12U) {
    if (_state != State::SEARCH_PREAMBLE) {
      resetToSearch();
    }
    return false;
  }

  if (_state == State::SEARCH_PREAMBLE) {
    // Valid TPMS half-bit periods span ~18 us (~28 kbps) up to ~150 us (~3.3 kbps / Schrader OOK 120 us)
    if (d < 15U || d > 160U) {
      _preambleCount = 0;
      _preambleSumUs = 0;
      return false;
    }

    if (_preambleCount == 0U) {
      _preambleSumUs = d;
      _preambleCount = 1U;
      _lastLevel = lvl;
      return false;
    }

    const uint16_t avg = static_cast<uint16_t>(_preambleSumUs / _preambleCount);
    const uint16_t tol = static_cast<uint16_t>((avg * 35U) / 100U + 4U);
    if (lvl != _lastLevel && d >= (avg - tol) && d <= (avg + tol)) {
      _preambleSumUs += d;
      ++_preambleCount;
      _lastLevel = lvl;

      if (_preambleCount >= 8U) {
        _halfBitUs = static_cast<uint16_t>(_preambleSumUs / _preambleCount);
        _state = State::PREAMBLE_LOCK;
      }
    } else {
      // Restart preamble tracking from this pulse
      _preambleSumUs = d;
      _preambleCount = 1U;
      _lastLevel = lvl;
    }
    return false;
  }

  // PREAMBLE_LOCK or IN_FRAME
  const uint16_t shortMin = static_cast<uint16_t>((_halfBitUs * 55U) / 100U);
  const uint16_t shortMax = static_cast<uint16_t>((_halfBitUs * 145U) / 100U);
  const uint16_t longMax  = static_cast<uint16_t>((_halfBitUs * 255U) / 100U);
  const uint16_t syncMax  = static_cast<uint16_t>((_halfBitUs * 360U) / 100U);

  if (d >= shortMin && d <= shortMax) {
    if (_state == State::PREAMBLE_LOCK) {
      // Refine half-bit average while still in alternating preamble
      _preambleSumUs += d;
      ++_preambleCount;
      _halfBitUs = static_cast<uint16_t>(_preambleSumUs / _preambleCount);
    } else {
      pushRawChip(lvl);
      if (_chipCount >= kMaxFrameChips) {
        return finalizeFrame(freqMhz, mod, rssiDbm, out);
      }
    }
    return false;
  }

  if (d > shortMax && d <= longMax) {
    if (_state == State::PREAMBLE_LOCK) {
      // First 2T pulse marks transition out of preamble into sync / frame payload
      _state = State::IN_FRAME;
      _chipCount = 0;
    }
    pushRawChip(lvl);
    pushRawChip(lvl);
    return (_chipCount >= kMaxFrameChips) ? finalizeFrame(freqMhz, mod, rssiDbm, out) : false;
  }

  // Handle 3T sync delimiter when transitioning from preamble into frame
  if (_state == State::PREAMBLE_LOCK && d > longMax && d <= syncMax) {
    _state = State::IN_FRAME;
    _chipCount = 0;
    return false;
  }

  // End-of-frame gap or out-of-window pulse
  if (_state == State::IN_FRAME && _chipCount >= (kMinFrameBits * 2U)) {
    return finalizeFrame(freqMhz, mod, rssiDbm, out);
  }

  resetToSearch();
  return false;
}

bool TpmsDecoder::finalizeFrame(float freqMhz, RfModulation mod, float rssiDbm, TpmsPacket& out) {
  if (_chipCount < (kMinFrameBits * 2U)) {
    resetToSearch();
    return false;
  }

  memset(&out, 0, sizeof(out));
  out.freq_mhz = freqMhz;
  strlcpy(out.modulation, (mod == RfModulation::OOK) ? "OOK" : "FSK", sizeof(out.modulation));
  out.rssi_dbm = rssiDbm;
  out.bitrate_bps = (_halfBitUs > 0U) ? (500000UL / _halfBitUs) : 0UL;
  out.timestamp_ms = millis();

  const uint16_t maxStart =
      (_chipCount > (kMinFrameBits * 2U))
          ? min<uint16_t>(24U, static_cast<uint16_t>(_chipCount - (kMinFrameBits * 2U)))
          : 0U;

  uint8_t bits[kMaxFrameBits]{};
  uint8_t bestBits[kMaxFrameBits]{};
  uint16_t bestCount = 0U;

  // Scan starting chip alignment (handles 1-chip half-bit sync offsets like Ford 0xAA 0xA9 and Schrader 4-bit sync)
  for (uint16_t startChip = 0U; startChip <= maxStart; ++startChip) {
    const uint16_t bitCount = extractManchester(_chips, _chipCount, startChip, bits, kMaxFrameBits);
    if (bitCount > bestCount) {
      bestCount = bitCount;
      memcpy(bestBits, bits, bitCount);
    }
    if (bitCount < kMinFrameBits) {
      continue;
    }

    uint8_t bytes[16]{};
    uint8_t byteLen = 0;
    packBits(bits, bitCount, bytes, byteLen);
    out.bit_count = bitCount;
    out.raw_len = byteLen;
    memcpy(out.raw_bytes, bytes, byteLen);

    if (tryDecodeAllPolars(bytes, bitCount, out)) {
      ++_decodedCount;
      resetToSearch();
      return true;
    }
  }

  // Fallback: Differential Manchester (used by Pacific / Toyota PMV-C210 over live RF)
  for (uint16_t startChip = 1U; startChip <= maxStart; ++startChip) {
    const uint16_t bitCount = extractDiffManchester(_chips, _chipCount, startChip, bits, kMaxFrameBits);
    if (bitCount < kMinFrameBits) {
      continue;
    }
    uint8_t bytes[16]{};
    uint8_t byteLen = 0;
    packBits(bits, bitCount, bytes, byteLen);
    out.bit_count = bitCount;
    out.raw_len = byteLen;
    memcpy(out.raw_bytes, bytes, byteLen);

    if (tryDecodeAllPolars(bytes, bitCount, out)) {
      ++_decodedCount;
      resetToSearch();
      return true;
    }
  }

  ++_rawCandidateCount;
  if (_rawMode && bestCount >= kMinFrameBits) {
    uint8_t bytes[16]{};
    uint8_t byteLen = 0;
    packBits(bestBits, bestCount, bytes, byteLen);
    out.bit_count = bestCount;
    out.raw_len = byteLen;
    memcpy(out.raw_bytes, bytes, byteLen);
    strlcpy(out.protocol, "RAW_MANCHESTER", sizeof(out.protocol));
    out.sensor_id = (byteLen >= 4U)
                        ? ((static_cast<uint32_t>(bytes[0]) << 24) |
                           (static_cast<uint32_t>(bytes[1]) << 16) |
                           (static_cast<uint32_t>(bytes[2]) << 8) |
                           static_cast<uint32_t>(bytes[3]))
                        : 0UL;
    out.crc_ok = false;
    resetToSearch();
    return true;
  }

  resetToSearch();
  return false;
}

bool TpmsDecoder::tryDecodeAllPolars(const uint8_t* bytes, uint16_t bitCount, TpmsPacket& out) const {
  if (tryDecodeSchrader(bytes, bitCount, out) ||
      tryDecodeToyota(bytes, bitCount, out) ||
      tryDecodeFord(bytes, bitCount, out) ||
      tryDecodeContinental(bytes, bitCount, out)) {
    return true;
  }

  // Test bitwise-inverted polarity (handles opposite FSK deviation or G.E. Thomas vs IEEE polarity)
  uint8_t inv[16]{};
  const uint8_t byteLen = static_cast<uint8_t>((bitCount + 7U) / 8U);
  for (uint8_t i = 0; i < byteLen && i < 16U; ++i) {
    inv[i] = static_cast<uint8_t>(~bytes[i]);
  }
  if (tryDecodeSchrader(inv, bitCount, out) ||
      tryDecodeToyota(inv, bitCount, out) ||
      tryDecodeFord(inv, bitCount, out) ||
      tryDecodeContinental(inv, bitCount, out)) {
    memcpy(out.raw_bytes, inv, out.raw_len);
    return true;
  }

  return false;
}

bool TpmsDecoder::tryDecodeSchrader(const uint8_t* b, uint16_t bitCount, TpmsPacket& out) const {
  // Schrader 64-bit frame: [0]=0xFx header/flags, [1..4]=ID, [5]=pressure(2.5 kPa), [6]=temp(C+50), [7]=CRC8(0x07, init 0xF0 or 0x00)
  if (bitCount >= 64U && bitCount <= 76U) {
    const uint8_t crcF0 = crc8(b, 7U, 0x07U, 0xF0U);
    const uint8_t crc00 = crc8(b, 7U, 0x07U, 0x00U);
    if ((b[0] & 0xF0U) == 0xF0U && (b[7] == crcF0 || b[7] == crc00)) {
      const uint32_t id = (static_cast<uint32_t>(b[1]) << 24) |
                          (static_cast<uint32_t>(b[2]) << 16) |
                          (static_cast<uint32_t>(b[3]) << 8) |
                          static_cast<uint32_t>(b[4]);
      if (id != 0UL && id != 0xFFFFFFFFUL) {
        strlcpy(out.protocol, "Schrader", sizeof(out.protocol));
        out.sensor_id = id;
        out.pressure_kpa = static_cast<float>(b[5]) * 2.5f;
        out.pressure_psi = out.pressure_kpa * 0.1450377f;
        out.temp_c = static_cast<float>(b[6]) - 50.0f;
        out.battery_low = (b[0] & 0x04U) != 0U;
        out.motion = (b[0] & 0x02U) != 0U;
        out.fast_leak = (b[0] & 0x01U) != 0U;
        out.crc_ok = true;
        return true;
      }
    }
  }

  // Schrader EG53MA4 80-bit frame: [0..1]=flags/header, [2..5]=32-bit ID, [6]=pressure(2.5 kPa), [7]=temp(C+50), [8]=status, [9]=sum8
  if (bitCount >= 80U && bitCount <= 88U) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 9U; ++i) {
      sum += b[i];
    }
    if ((b[0] & 0xF0U) == 0xF0U && (sum & 0xFFU) == b[9]) {
      const uint32_t id = (static_cast<uint32_t>(b[2]) << 24) |
                          (static_cast<uint32_t>(b[3]) << 16) |
                          (static_cast<uint32_t>(b[4]) << 8) |
                          static_cast<uint32_t>(b[5]);
      if (id != 0UL && id != 0xFFFFFFFFUL) {
        strlcpy(out.protocol, "Schrader-80", sizeof(out.protocol));
        out.sensor_id = id;
        out.pressure_kpa = static_cast<float>(b[6]) * 2.5f;
        out.pressure_psi = out.pressure_kpa * 0.1450377f;
        out.temp_c = static_cast<float>(b[7]) - 50.0f;
        out.battery_low = (b[1] & 0x04U) != 0U;
        out.motion = (b[1] & 0x02U) != 0U;
        out.fast_leak = (b[1] & 0x01U) != 0U;
        out.crc_ok = true;
        return true;
      }
    }
  }

  return false;
}

bool TpmsDecoder::tryDecodeToyota(const uint8_t* b, uint16_t bitCount, TpmsPacket& out) const {
  // Toyota PMV-C210 72-bit frame: [0..3]=ID, [4]=status, [5]=pressure(0.25 PSI - 7), [6]=temp(C+40), [7]=~b[5], [8]=CRC8(0x07, init 0x80)
  if (bitCount < 72U || bitCount > 84U) {
    return false;
  }

  if (b[7] != static_cast<uint8_t>(~b[5])) {
    return false;
  }

  if (crc8(b, 8U, 0x07U, 0x80U) != b[8]) {
    return false;
  }

  const uint32_t id = (static_cast<uint32_t>(b[0]) << 24) |
                      (static_cast<uint32_t>(b[1]) << 16) |
                      (static_cast<uint32_t>(b[2]) << 8) |
                      static_cast<uint32_t>(b[3]);
  if (id == 0UL || id == 0xFFFFFFFFUL) {
    return false;
  }

  strlcpy(out.protocol, "Toyota", sizeof(out.protocol));
  out.sensor_id = id;
  out.pressure_psi = (static_cast<float>(b[5]) * 0.25f) - 7.0f;
  if (out.pressure_psi < 0.0f) {
    out.pressure_psi = 0.0f;
  }
  out.pressure_kpa = out.pressure_psi * 6.894757f;
  out.temp_c = static_cast<float>(b[6]) - 40.0f;
  out.battery_low = (b[4] & 0x80U) != 0U;
  out.motion = (b[4] & 0x20U) != 0U;
  out.fast_leak = (b[4] & 0x10U) != 0U;
  out.crc_ok = true;
  return true;
}

bool TpmsDecoder::tryDecodeFord(const uint8_t* b, uint16_t bitCount, TpmsPacket& out) const {
  // Ford 64-bit frame: [0..3]=ID, [4]=pressure LSB (0.25 PSI), [5]=temp(C+56), [6]=flags(bit5=+256 pressure), [7]=sum8
  if (bitCount < 64U || bitCount > 76U) {
    return false;
  }

  uint16_t sum = 0;
  for (uint8_t i = 0; i < 7U; ++i) {
    sum += b[i];
  }
  if ((sum & 0xFFU) != b[7]) {
    return false;
  }

  const uint32_t id = (static_cast<uint32_t>(b[0]) << 24) |
                      (static_cast<uint32_t>(b[1]) << 16) |
                      (static_cast<uint32_t>(b[2]) << 8) |
                      static_cast<uint32_t>(b[3]);
  if (id == 0UL || id == 0xFFFFFFFFUL) {
    return false;
  }

  const uint16_t rawPress = static_cast<uint16_t>(b[4]) |
                            ((b[6] & 0x20U) ? 0x0100U : 0x0000U);
  if (rawPress > 400U) {
    return false;
  }

  // Per rtl_433 tpms_ford.c: b[6] & 0x4C must be 0x08 (LF learn), 0x04 (at rest), or 0x44 (moving);
  // 0x80 and 0x10 are always 0 on genuine packets.
  const uint8_t modeBits = b[6] & 0x4CU;
  if ((b[6] & 0x90U) != 0U ||
      (modeBits != 0x08U && modeBits != 0x04U && modeBits != 0x44U)) {
    return false;
  }

  strlcpy(out.protocol, "Ford", sizeof(out.protocol));
  out.sensor_id = id;
  out.pressure_psi = static_cast<float>(rawPress) * 0.25f;
  out.pressure_kpa = out.pressure_psi * 6.894757f;
  out.temp_c = ((b[5] & 0x80U) == 0U) ? (static_cast<float>(b[5] & 0x7FU) - 56.0f) : 0.0f;
  out.battery_low = false;  // Ford 8-byte frame has no low-battery bit (0x08 is LF learn)
  out.motion = (modeBits == 0x44U);
  out.fast_leak = false;    // 0x02 is status telemetry, not a leak alarm
  out.crc_ok = true;
  return true;
}

bool TpmsDecoder::tryDecodeContinental(const uint8_t* b, uint16_t bitCount, TpmsPacket& out) const {
  // Continental/VDO 64-bit frame: [0..3]=32-bit ID, [4]=pressure(2.5 kPa), [5]=temp(C+50), [6]=flags, [7]=CRC8(0x07)
  if (bitCount < 64U || bitCount > 76U) {
    return false;
  }

  if ((b[0] & 0xF0U) == 0xF0U) {
    return false;
  }

  if (crc8(b, 7U, 0x07U, 0x00U) != b[7]) {
    return false;
  }

  const uint32_t id = (static_cast<uint32_t>(b[0]) << 24) |
                      (static_cast<uint32_t>(b[1]) << 16) |
                      (static_cast<uint32_t>(b[2]) << 8) |
                      static_cast<uint32_t>(b[3]);
  if (id == 0UL || id == 0xFFFFFFFFUL) {
    return false;
  }

  strlcpy(out.protocol, "Continental", sizeof(out.protocol));
  out.sensor_id = id;
  out.pressure_kpa = static_cast<float>(b[4]) * 2.5f;
  out.pressure_psi = out.pressure_kpa * 0.1450377f;
  out.temp_c = static_cast<float>(b[5]) - 50.0f;
  out.battery_low = (b[6] & 0x04U) != 0U;
  out.motion = (b[6] & 0x02U) != 0U;
  out.fast_leak = (b[6] & 0x01U) != 0U;
  out.crc_ok = true;
  return true;
}

bool TpmsDecoder::injectTestPacket(uint8_t protoIndex,
                                   float freqMhz,
                                   RfModulation mod,
                                   float rssiDbm,
                                   TpmsPacket& out) {
  resetToSearch();

  uint8_t frame[9]{};
  uint8_t frameBytes = 8U;

  if (protoIndex == 1U) {
    // Toyota PMV-C210 72-bit frame: ID=0x89ABCDEF, 34.0 PSI, 27.0 C, motion=true
    frame[0] = 0x89;
    frame[1] = 0xAB;
    frame[2] = 0xCD;
    frame[3] = 0xEF;
    frame[4] = 0x20;
    frame[5] = 0xA4;
    frame[6] = 0x43;
    frame[7] = static_cast<uint8_t>(~frame[5]);
    frame[8] = crc8(frame, 8U, 0x07U, 0x80U);
    frameBytes = 9U;
  } else if (protoIndex == 2U) {
    // Ford 64-bit frame: ID=0x0456789A, 35.0 PSI, 22.0 C, motion=true (0x44 per tpms_ford.c)
    frame[0] = 0x04;
    frame[1] = 0x56;
    frame[2] = 0x78;
    frame[3] = 0x9A;
    frame[4] = 0x8C;
    frame[5] = 0x4E;
    frame[6] = 0x44;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 7U; ++i) {
      sum += frame[i];
    }
    frame[7] = static_cast<uint8_t>(sum & 0xFFU);
    frameBytes = 8U;
  } else {
    // Schrader 64-bit frame: ID=0x01A2B3C4, 225.0 kPa (32.6 PSI), 24.0 C, motion=true
    frame[0] = 0xF2;
    frame[1] = 0x01;
    frame[2] = 0xA2;
    frame[3] = 0xB3;
    frame[4] = 0xC4;
    frame[5] = 0x5A;
    frame[6] = 0x4A;
    frame[7] = crc8(frame, 7U, 0x07U, 0x00U);
    frameBytes = 8U;
  }

  // Convert preamble + sync + Manchester payload into coalesced GDO0 pulses at 19.2 kbps (T = 26 us)
  constexpr uint16_t kHalfUs = 26U;
  uint8_t chips[256]{};
  uint16_t chipCount = 0;

  // 16 preamble half-bits (10101010...)
  for (uint8_t i = 0; i < 16U; ++i) {
    chips[chipCount++] = (i & 0x01U) ? 0U : 1U;
  }
  // 2T sync delimiter without merging into a 3T pulse against the payload's leading chip
  chips[chipCount++] = 1U;
  chips[chipCount++] = 1U;
  if ((frame[0] & 0x80U) != 0U) {
    chips[chipCount++] = 0U;
    chips[chipCount++] = 0U;
  }

  // Manchester encode payload bits: bit 1 -> chips (1,0), bit 0 -> chips (0,1)
  for (uint8_t byteIdx = 0; byteIdx < frameBytes; ++byteIdx) {
    for (int8_t bit = 7; bit >= 0; --bit) {
      const uint8_t b = (frame[byteIdx] >> bit) & 0x01U;
      chips[chipCount++] = b ? 1U : 0U;
      chips[chipCount++] = b ? 0U : 1U;
    }
  }

  // Coalesce consecutive identical chips into pulse durations
  bool decoded = false;
  uint16_t idx = 0;
  while (idx < chipCount) {
    const uint8_t level = chips[idx];
    uint16_t run = 1U;
    while ((idx + run) < chipCount && chips[idx + run] == level) {
      ++run;
    }
    const PulseEvent p{static_cast<uint16_t>(run * kHalfUs), level, 0};
    if (feedPulse(p, freqMhz, mod, rssiDbm, out)) {
      decoded = true;
    }
    idx += run;
  }

  // End-of-frame idle gap
  const PulseEvent gap{5000U, 0U, 0U};
  if (feedPulse(gap, freqMhz, mod, rssiDbm, out)) {
    decoded = true;
  }
  return decoded;
}
