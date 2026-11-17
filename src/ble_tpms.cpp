// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#include "ble_tpms.h"
#include "config.h"
#include <math.h>
#include <string.h>

namespace {

BleTpmsReceiver* s_activeBleTpms = nullptr;

uint32_t fnv1aHash(const uint8_t* data, uint8_t len) {
  uint32_t h = 2166136261UL;
  for (uint8_t i = 0; i < len; ++i) {
    h ^= data[i];
    h *= 16777619UL;
  }
  return h;
}

template <typename EntryT>
bool shouldEmitDedup(EntryT* cache,
                     size_t cacheLen,
                     uint8_t& nextIdx,
                     uint32_t key,
                     uint32_t hash,
                     uint32_t nowMs,
                     uint32_t windowMs) {
  for (size_t i = 0; i < cacheLen; ++i) {
    if (cache[i].key == key) {
      if (cache[i].hash == hash && (nowMs - cache[i].lastMs) < windowMs) {
        return false;
      }
      cache[i].hash = hash;
      cache[i].lastMs = nowMs;
      return true;
    }
  }
  cache[nextIdx].key = key;
  cache[nextIdx].hash = hash;
  cache[nextIdx].lastMs = nowMs;
  nextIdx = static_cast<uint8_t>((nextIdx + 1U) % cacheLen);
  return true;
}

bool matchMacReversed(const uint8_t* addrLe, const uint8_t* bytesBe6) {
  for (uint8_t i = 0; i < 6U; ++i) {
    if (addrLe[5U - i] != bytesBe6[i]) {
      return false;
    }
  }
  return true;
}

uint32_t readU32Le(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

// Bounds-safe AD TLV walker (guards against malformed lengths in air traffic)
void parseAdPayload(const uint8_t* data,
                    uint16_t len,
                    char* nameOut,
                    size_t nameCap,
                    const uint8_t*& msdOut,
                    uint8_t& msdLenOut) {
  nameOut[0] = '\0';
  msdOut = nullptr;
  msdLenOut = 0;

  uint16_t offset = 0;
  while (offset + 1U < len) {
    const uint8_t fieldLen = data[offset];
    if (fieldLen == 0U || (offset + 1U + fieldLen) > len) {
      break;
    }
    const uint8_t adType = data[offset + 1U];
    const uint8_t valueLen = static_cast<uint8_t>(fieldLen - 1U);
    const uint8_t* valuePtr = &data[offset + 2U];

    if (adType == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA && valueLen > 0U) {
      msdOut = valuePtr;
      msdLenOut = valueLen;
    } else if ((adType == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
                adType == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME) &&
               valueLen > 0U && nameCap > 1U) {
      const size_t copyLen = (valueLen < (nameCap - 1U)) ? valueLen : (nameCap - 1U);
      memcpy(nameOut, valuePtr, copyLen);
      nameOut[copyLen] = '\0';
    }

    offset = static_cast<uint16_t>(offset + 1U + fieldLen);
  }
}

void populateBaseBlePacket(const char* protoName,
                           uint32_t sensorId,
                           const uint8_t* msd,
                           uint8_t msdLen,
                           int8_t rssi,
                           uint32_t nowMs,
                           TpmsPacket& out) {
  memset(&out, 0, sizeof(out));
  strncpy(out.protocol, protoName, sizeof(out.protocol) - 1U);
  out.sensor_id = sensorId;
  out.freq_mhz = 2402.0f;
  strncpy(out.modulation, "BLE", sizeof(out.modulation) - 1U);
  out.rssi_dbm = static_cast<float>(rssi);
  out.bit_count = static_cast<uint16_t>(msdLen * 8U);
  out.bitrate_bps = 1000000UL;
  const uint8_t copyLen = (msdLen > sizeof(out.raw_bytes)) ? sizeof(out.raw_bytes) : msdLen;
  memcpy(out.raw_bytes, msd, copyLen);
  out.raw_len = copyLen;
  out.timestamp_ms = nowMs;
}

}  // namespace

void BleTpmsReceiver::begin(bool enabled) {
  s_activeBleTpms = this;

  Bluefruit.Scanner.setRxCallback(onScanReport);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.setIntervalMS(BLE_SCAN_INTERVAL_MS, BLE_SCAN_WINDOW_MS);
  Bluefruit.Scanner.clearFilters();

  setEnabled(enabled);
}

void BleTpmsReceiver::setEnabled(bool enabled) {
  _enabled = enabled;
  if (enabled) {
    if (!Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.start(0);
    }
  } else {
    if (Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.stop();
    }
  }
}

bool BleTpmsReceiver::isScannerRunning() const {
  return Bluefruit.Scanner.isRunning();
}

void BleTpmsReceiver::onScanReport(ble_gap_evt_adv_report_t* report) {
  if (s_activeBleTpms && report) {
    s_activeBleTpms->handleReport(report);
  }
  Bluefruit.Scanner.resume();
}

void BleTpmsReceiver::handleReport(const ble_gap_evt_adv_report_t* report) {
  if (!_enabled) {
    return;
  }

  ++_advCount;

  char localName[16]{};
  const uint8_t* msd = nullptr;
  uint8_t msdLen = 0;
  parseAdPayload(report->data.p_data, report->data.len, localName, sizeof(localName), msd, msdLen);
  if (!msd || msdLen < 4U) {
    return;
  }

  const uint32_t nowMs = millis();
  TpmsPacket pkt{};
  if (decodeManufacturerData(
          report->peer_addr.addr, localName, msd, msdLen, report->rssi, nowMs, pkt)) {
    enqueuePacket(pkt);
  } else if (_rawMode && report->rssi >= BLE_RAW_MIN_RSSI_DBM) {
    enqueueRawAdv(report->peer_addr.addr, localName, msd, msdLen, report->rssi, nowMs);
  }
}

bool BleTpmsReceiver::decodeManufacturerData(const uint8_t* peerAddr,
                                             const char* localName,
                                             const uint8_t* msd,
                                             uint8_t msdLen,
                                             int8_t rssi,
                                             uint32_t nowMs,
                                             TpmsPacket& out) {
  if (decodeFormat0100(peerAddr, msd, msdLen, rssi, nowMs, out)) {
    return true;
  }
  if (decodeFormatBr(peerAddr, localName, msd, msdLen, rssi, nowMs, out)) {
    return true;
  }
  return false;
}

bool BleTpmsReceiver::decodeFormat0100(const uint8_t* peerAddr,
                                       const uint8_t* msd,
                                       uint8_t msdLen,
                                       int8_t rssi,
                                       uint32_t nowMs,
                                       TpmsPacket& out) {
  // Sysgration / TomTom / Zeepin 18-byte MSD:
  // [0..1]=0x00,0x01 (or 0x01,0x00) [2..7]=MAC(8x:EA:CA:xx:yy:zz)
  // [8..11]=pressure_Pa_LE [12..15]=temp_cdeg_LE [16]=batt_pct [17]=alarm
  if (msdLen < 18U) {
    return false;
  }
  const bool validCid = (msd[0] == 0x00U && msd[1] == 0x01U) ||
                        (msd[0] == 0x01U && msd[1] == 0x00U);
  if (!validCid) {
    return false;
  }

  const bool macMatch = peerAddr ? matchMacReversed(peerAddr, &msd[2]) : false;
  const bool eacaMatch = (msd[3] == 0xEAU && msd[4] == 0xCAU && msd[16] <= 100U);
  if (!macMatch && !eacaMatch) {
    return false;
  }

  const uint32_t rawPa = readU32Le(&msd[8]);
  const int32_t rawCdeg = static_cast<int32_t>(readU32Le(&msd[12]));
  if (rawPa > 1500000UL || rawCdeg < -5000 || rawCdeg > 12500) {
    return false;
  }

  const uint32_t sensorId = (static_cast<uint32_t>(msd[4]) << 24) |
                            (static_cast<uint32_t>(msd[5]) << 16) |
                            (static_cast<uint32_t>(msd[6]) << 8) |
                            static_cast<uint32_t>(msd[7]);

  populateBaseBlePacket("BLE_0100", sensorId, msd, msdLen, rssi, nowMs, out);
  out.pressure_kpa = static_cast<float>(rawPa) / 1000.0f;
  out.pressure_psi = out.pressure_kpa * 0.1450377f;
  out.temp_c = static_cast<float>(rawCdeg) / 100.0f;
  out.battery_low = (msd[16] <= 20U);
  out.motion = false;
  out.fast_leak = (msd[17] != 0U);
  out.crc_ok = true;
  return true;
}

bool BleTpmsReceiver::decodeFormatBr(const uint8_t* peerAddr,
                                     const char* localName,
                                     const uint8_t* msd,
                                     uint8_t msdLen,
                                     int8_t rssi,
                                     uint32_t nowMs,
                                     TpmsPacket& out) {
  // White-label "BR" / SYTPMS sensors (name "BR*" or MAC AC:15:85:xx:yy:zz)
  if (!msd || msdLen < 5U) {
    return false;
  }
  const bool nameIsBr = (localName && localName[0] == 'B' && localName[1] == 'R');
  const bool macIsAc1585 = (peerAddr &&
                            peerAddr[5] == 0xACU &&
                            peerAddr[4] == 0x15U &&
                            peerAddr[3] == 0x85U);
  if (!nameIsBr && !macIsAc1585) {
    return false;
  }

  float vbatV = 3.0f;
  float tempC = 0.0f;
  float absPsi = 0.0f;
  if (msdLen >= 7U && msd[1] >= 18U && msd[1] <= 36U) {
    // 7-byte MSD layout: [status][vbat_0.1V][temp_C][p_hi][p_lo][..] (0.1 PSI absolute)
    vbatV = static_cast<float>(msd[1]) * 0.1f;
    tempC = static_cast<float>(static_cast<int8_t>(msd[2]));
    const uint16_t rawP = static_cast<uint16_t>((msd[3] << 8) | msd[4]);
    absPsi = static_cast<float>(rawP) * 0.1f;
  } else if (msd[0] >= 18U && msd[0] <= 36U) {
    // Shifted MSD layout: [vbat_0.1V][temp_C][p_hi][p_lo] (0.1 kPa absolute -> convert to PSI)
    vbatV = static_cast<float>(msd[0]) * 0.1f;
    tempC = static_cast<float>(static_cast<int8_t>(msd[1]));
    const uint16_t rawKpa01 = static_cast<uint16_t>((msd[2] << 8) | msd[3]);
    absPsi = (static_cast<float>(rawKpa01) * 0.1f) * 0.1450377f;
  } else {
    return false;
  }

  const float gaugePsi = (absPsi > 14.504f) ? (absPsi - 14.504f) : 0.0f;
  uint32_t sensorId = 0xB8000000UL;
  if (peerAddr) {
    sensorId = (static_cast<uint32_t>(peerAddr[3]) << 24) |
               (static_cast<uint32_t>(peerAddr[2]) << 16) |
               (static_cast<uint32_t>(peerAddr[1]) << 8) |
               static_cast<uint32_t>(peerAddr[0]);
  }

  populateBaseBlePacket("BLE_BR", sensorId, msd, msdLen, rssi, nowMs, out);
  out.pressure_psi = gaugePsi;
  out.pressure_kpa = gaugePsi / 0.1450377f;
  out.temp_c = tempC;
  out.battery_low = (vbatV < 2.5f);
  out.motion = false;
  out.fast_leak = false;
  out.crc_ok = false;
  return true;
}

void BleTpmsReceiver::enqueuePacket(const TpmsPacket& pkt) {
  const uint32_t hash = fnv1aHash(pkt.raw_bytes, pkt.raw_len);
  if (!shouldEmitDedup(_pktDedup, 8U, _pktDedupNext, pkt.sensor_id, hash, pkt.timestamp_ms, 400U)) {
    return;
  }

  const uint8_t next = static_cast<uint8_t>((_pktHead + 1U) & (BLE_TPMS_RING_SIZE - 1U));
  if (next == _pktTail) {
    return;
  }
  _pktRing[_pktHead] = pkt;
  __DMB();
  _pktHead = next;
  ++_decodedCount;
}

void BleTpmsReceiver::enqueueRawAdv(const uint8_t* peerAddr,
                                    const char* localName,
                                    const uint8_t* msd,
                                    uint8_t msdLen,
                                    int8_t rssi,
                                    uint32_t nowMs) {
  const uint32_t macKey = (static_cast<uint32_t>(peerAddr[3]) << 24) |
                          (static_cast<uint32_t>(peerAddr[2]) << 16) |
                          (static_cast<uint32_t>(peerAddr[1]) << 8) |
                          static_cast<uint32_t>(peerAddr[0]);
  const uint32_t hash = fnv1aHash(msd, msdLen);
  if (!shouldEmitDedup(_rawDedup, 8U, _rawDedupNext, macKey, hash, nowMs, 2000U)) {
    return;
  }

  const uint8_t next = static_cast<uint8_t>((_rawHead + 1U) & (BLE_RAW_RING_SIZE - 1U));
  if (next == _rawTail) {
    return;
  }
  BleRawAdvEvent& slot = _rawRing[_rawHead];
  memset(&slot, 0, sizeof(slot));
  memcpy(slot.addr, peerAddr, 6U);
  if (localName) {
    strncpy(slot.name, localName, sizeof(slot.name) - 1U);
  }
  const uint8_t copyLen = (msdLen > sizeof(slot.msd)) ? sizeof(slot.msd) : msdLen;
  memcpy(slot.msd, msd, copyLen);
  slot.msd_len = copyLen;
  slot.rssi_dbm = rssi;
  slot.timestamp_ms = nowMs;
  __DMB();
  _rawHead = next;
}

bool BleTpmsReceiver::popPacket(TpmsPacket& out) {
  if (_pktTail == _pktHead) {
    return false;
  }
  __DMB();
  out = _pktRing[_pktTail];
  __DMB();
  _pktTail = static_cast<uint8_t>((_pktTail + 1U) & (BLE_TPMS_RING_SIZE - 1U));
  return true;
}

bool BleTpmsReceiver::popRawAdv(BleRawAdvEvent& out) {
  if (_rawTail == _rawHead) {
    return false;
  }
  __DMB();
  out = _rawRing[_rawTail];
  __DMB();
  _rawTail = static_cast<uint8_t>((_rawTail + 1U) & (BLE_RAW_RING_SIZE - 1U));
  return true;
}

bool BleTpmsReceiver::injectTestPacket(TpmsPacket& out) {
  // Synthetic 18-byte 0x0100 BLE TPMS advertisement:
  // MAC 80:EA:CA:55:21:04, Pressure = 241300 Pa (241.3 kPa / 35.0 PSI), Temp = 2400 (24.0 C), Batt = 92%
  static constexpr uint8_t kAddr[6] = {0x04, 0x21, 0x55, 0xCA, 0xEA, 0x80};
  static constexpr uint8_t kMsd[18] = {
      0x00, 0x01, 0x80, 0xEA, 0xCA, 0x55, 0x21, 0x04,
      0x94, 0xAE, 0x03, 0x00, 0x60, 0x09, 0x00, 0x00,
      0x5C, 0x00
  };
  return decodeManufacturerData(kAddr, "TPMS1", kMsd, sizeof(kMsd), -61, millis(), out);
}
