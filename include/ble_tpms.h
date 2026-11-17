// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>
#include <bluefruit.h>
#include "config.h"
#include "tpms_decoder.h"

struct BleRawAdvEvent {
  uint8_t  addr[6];
  char     name[16];
  uint8_t  msd[20];
  uint8_t  msd_len;
  int8_t   rssi_dbm;
  uint32_t timestamp_ms;
};

class BleTpmsReceiver {
 public:
  void begin(bool enabled = true);

  void setEnabled(bool enabled);
  bool isEnabled() const { return _enabled; }
  bool isScannerRunning() const;

  void setRawMode(bool enabled) { _rawMode = enabled; }
  bool rawMode() const { return _rawMode; }

  bool popPacket(TpmsPacket& out);
  bool popRawAdv(BleRawAdvEvent& out);

  // Feeds a synthetic 18-byte 0x0100 BLE TPMS Manufacturer Data payload through the real decoder.
  bool injectTestPacket(TpmsPacket& out);

  uint32_t advCount() const { return _advCount; }
  uint32_t decodedCount() const { return _decodedCount; }

  static bool decodeManufacturerData(const uint8_t* peerAddr,
                                     const char* localName,
                                     const uint8_t* msd,
                                     uint8_t msdLen,
                                     int8_t rssi,
                                     uint32_t nowMs,
                                     TpmsPacket& out);

 private:
  static void onScanReport(ble_gap_evt_adv_report_t* report);
  void handleReport(const ble_gap_evt_adv_report_t* report);

  static bool decodeFormat0100(const uint8_t* peerAddr,
                               const uint8_t* msd,
                               uint8_t msdLen,
                               int8_t rssi,
                               uint32_t nowMs,
                               TpmsPacket& out);
  static bool decodeFormatBr(const uint8_t* peerAddr,
                             const char* localName,
                             const uint8_t* msd,
                             uint8_t msdLen,
                             int8_t rssi,
                             uint32_t nowMs,
                             TpmsPacket& out);

  void enqueuePacket(const TpmsPacket& pkt);
  void enqueueRawAdv(const uint8_t* peerAddr,
                     const char* localName,
                     const uint8_t* msd,
                     uint8_t msdLen,
                     int8_t rssi,
                     uint32_t nowMs);

  struct DedupEntry {
    uint32_t key;
    uint32_t hash;
    uint32_t lastMs;
  };

  bool _enabled = false;
  bool _rawMode = false;

  volatile uint32_t _advCount     = 0;
  volatile uint32_t _decodedCount = 0;

  TpmsPacket        _pktRing[BLE_TPMS_RING_SIZE]{};
  volatile uint8_t  _pktHead = 0;
  volatile uint8_t  _pktTail = 0;

  BleRawAdvEvent    _rawRing[BLE_RAW_RING_SIZE]{};
  volatile uint8_t  _rawHead = 0;
  volatile uint8_t  _rawTail = 0;

  DedupEntry        _rawDedup[8]{};
  uint8_t           _rawDedupNext = 0;
  DedupEntry        _pktDedup[8]{};
  uint8_t           _pktDedupNext = 0;
};
