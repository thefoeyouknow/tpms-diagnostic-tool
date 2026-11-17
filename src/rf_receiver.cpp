#include "rf_receiver.h"
#include "config.h"
#include <SPI.h>
#include <RadioLib.h>

namespace {

Module g_cc1101Module(
    PIN_CC1101_CSN,
    PIN_CC1101_GDO0,
    RADIOLIB_NC,
    PIN_CC1101_GDO2,
    SPI);

CC1101 g_radio(&g_cc1101Module);

volatile PulseEvent s_pulseRing[PULSE_RING_SIZE];
volatile uint16_t   s_ringHead      = 0;
volatile uint16_t   s_ringTail      = 0;
volatile uint32_t   s_totalPulses   = 0;
volatile uint32_t   s_droppedPulses = 0;
volatile uint32_t   s_lastEdgeUs    = 0;
volatile bool       s_captureArmed  = false;

inline uint32_t readTimer1Micros() {
  NRF_TIMER1->TASKS_CAPTURE[0] = 1;
  return NRF_TIMER1->CC[0];
}

uint8_t bbTransferByte(uint8_t tx) {
  uint8_t rx = 0;
  for (int8_t bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_CC1101_MOSI, (tx & (1U << bit)) ? HIGH : LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_CC1101_SCK, HIGH);
    if (digitalRead(PIN_CC1101_MISO) == HIGH) {
      rx |= static_cast<uint8_t>(1U << bit);
    }
    delayMicroseconds(5);
    digitalWrite(PIN_CC1101_SCK, LOW);
  }
  return rx;
}

bool waitMisoLow(uint32_t timeoutUs) {
  const uint32_t startUs = readTimer1Micros();
  while ((readTimer1Micros() - startUs) < timeoutUs) {
    if (digitalRead(PIN_CC1101_MISO) == LOW) {
      return true;
    }
  }
  return (digitalRead(PIN_CC1101_MISO) == LOW);
}

}  // namespace

void RfReceiver::initTimer1Micros() {
  // Enable DWT as well (used by core micros() when CPU is awake)
  dwt_enable();

  // NRF_TIMER1 runs continuously at 1 MHz even when FreeRTOS enters __WFE()
  NRF_TIMER1->TASKS_STOP = 1;
  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER1->PRESCALER = 4;  // 16 MHz / 2^4 = 1 MHz (1 us resolution)
  NRF_TIMER1->TASKS_START = 1;
}

void RfReceiver::onGdo0Edge() {
  if (!s_captureArmed) {
    return;
  }

  const uint32_t nowUs = readTimer1Micros();
  uint32_t dt = nowUs - s_lastEdgeUs;
  s_lastEdgeUs = nowUs;

  if (dt == 0U) {
    dt = 1U;
  } else if (dt > 65535UL) {
    dt = 65535UL;
  }

  // Level that just ended is the inverse of the current post-edge pin level
  const uint8_t newLevel = static_cast<uint8_t>((NRF_GPIO->IN >> PIN_CC1101_GDO0) & 0x01UL);
  const uint8_t endedLevel = newLevel ^ 0x01U;

  ++s_totalPulses;

  const uint16_t head = s_ringHead;
  const uint16_t next = static_cast<uint16_t>((head + 1U) & (PULSE_RING_SIZE - 1U));
  if (next == s_ringTail) {
    ++s_droppedPulses;
    return;
  }

  s_pulseRing[head].duration_us = static_cast<uint16_t>(dt);
  s_pulseRing[head].level = endedLevel;
  s_pulseRing[head].reserved = 0;
  __DMB();
  s_ringHead = next;
}

void RfReceiver::probeAndResetCc1101(bool runPinMatrixScan) {
  memset(&_hwDiag, 0, sizeof(_hwDiag));
  _hwDiag.hw_version = -1;
  _hwDiag.clk_pin = -1;
  _hwDiag.rdy_csn = -1;
  _hwDiag.rdy_miso = -1;

  SPI.end();

  if (runPinMatrixScan) {
    // 0. Full 17-pin auto-discovery matrix across all exposed Feather header GPIOs
    static constexpr uint8_t kScanPins[17] = {
        2, 3, 4, 5, 7, 11, 12, 13, 14, 15, 16, 25, 26, 27, 28, 29, 30};
    for (uint8_t i = 0; i < 17U; ++i) {
      pinMode(kScanPins[i], INPUT_PULLUP);
    }
    delayMicroseconds(50);

    // Clock scan: find pin carrying CC1101 default 135.4 kHz XOSC/192 clock
    for (uint8_t i = 0; i < 17U; ++i) {
      const uint8_t pin = kScanPins[i];
      uint32_t prev = (NRF_GPIO->IN >> pin) & 0x01UL;
      uint32_t edges = 0;
      const uint32_t startUs = readTimer1Micros();
      while ((readTimer1Micros() - startUs) < 2000UL) {
        const uint32_t cur = (NRF_GPIO->IN >> pin) & 0x01UL;
        if (cur != prev) {
          ++edges;
          prev = cur;
        }
      }
      if (edges > _hwDiag.clk_edges) {
        _hwDiag.clk_edges = edges;
        _hwDiag.clk_pin = static_cast<int8_t>(pin);
      }
    }
    if (_hwDiag.clk_edges == 0U) {
      _hwDiag.clk_pin = -1;
    }

    // CHIP_RDYn matrix scan: drive each candidate CSN HIGH->LOW->HIGH and check MISO response
    for (uint8_t i = 0; i < 17U && _hwDiag.rdy_csn < 0; ++i) {
      const uint8_t csnPin = kScanPins[i];
      pinMode(csnPin, OUTPUT);
      digitalWrite(csnPin, HIGH);
      delayMicroseconds(20);

      uint8_t highState[17]{};
      for (uint8_t j = 0; j < 17U; ++j) {
        highState[j] = static_cast<uint8_t>(digitalRead(kScanPins[j]));
      }

      digitalWrite(csnPin, LOW);
      delayMicroseconds(250);

      for (uint8_t j = 0; j < 17U; ++j) {
        if (j == i) {
          continue;
        }
        if (highState[j] == 1U && digitalRead(kScanPins[j]) == LOW) {
          delayMicroseconds(6);  // Filter out 135 kHz toggling clock
          if (digitalRead(kScanPins[j]) == LOW) {
            digitalWrite(csnPin, HIGH);
            delayMicroseconds(20);
            if (digitalRead(kScanPins[j]) == HIGH) {
              _hwDiag.rdy_csn = static_cast<int8_t>(csnPin);
              _hwDiag.rdy_miso = static_cast<int8_t>(kScanPins[j]);
              break;
            }
            digitalWrite(csnPin, LOW);
            delayMicroseconds(20);
          }
        }
      }

      pinMode(csnPin, INPUT_PULLUP);
      delayMicroseconds(20);
    }

    // Restore LF wake gate pin strictly LOW immediately after scan
    pinMode(PIN_LF_WAKE_PWM, OUTPUT);
    digitalWrite(PIN_LF_WAKE_PWM, LOW);
  }

  pinMode(PIN_CC1101_CSN, OUTPUT);
  digitalWrite(PIN_CC1101_CSN, HIGH);
  pinMode(PIN_CC1101_SCK, OUTPUT);
  digitalWrite(PIN_CC1101_SCK, LOW);
  pinMode(PIN_CC1101_MOSI, OUTPUT);
  digitalWrite(PIN_CC1101_MOSI, LOW);

  // 1. Check MISO High-Z integrity while CSN is deasserted (HIGH)
  pinMode(PIN_CC1101_MISO, INPUT_PULLUP);
  delayMicroseconds(50);
  _hwDiag.miso_pullup_cs_high = static_cast<uint8_t>(digitalRead(PIN_CC1101_MISO));

  pinMode(PIN_CC1101_MISO, INPUT_PULLDOWN);
  delayMicroseconds(50);
  _hwDiag.miso_pulldown_cs_high = static_cast<uint8_t>(digitalRead(PIN_CC1101_MISO));

  pinMode(PIN_CC1101_MISO, INPUT_PULLUP);

  // 2. Check GDO0 pull-up / pull-down state and count transitions over 2 ms
  pinMode(PIN_CC1101_GDO0, INPUT_PULLUP);
  delayMicroseconds(50);
  _hwDiag.gdo0_pu = static_cast<uint8_t>(digitalRead(PIN_CC1101_GDO0));

  pinMode(PIN_CC1101_GDO0, INPUT_PULLDOWN);
  delayMicroseconds(50);
  _hwDiag.gdo0_pd = static_cast<uint8_t>(digitalRead(PIN_CC1101_GDO0));

  pinMode(PIN_CC1101_GDO0, INPUT);
  uint32_t prevLvl = (NRF_GPIO->IN >> PIN_CC1101_GDO0) & 0x01UL;
  uint32_t edges = 0;
  const uint32_t t0 = readTimer1Micros();
  while ((readTimer1Micros() - t0) < 2000UL) {
    const uint32_t lvl = (NRF_GPIO->IN >> PIN_CC1101_GDO0) & 0x01UL;
    if (lvl != prevLvl) {
      ++edges;
      prevLvl = lvl;
    }
  }
  _hwDiag.gdo0_edges_2ms = edges;

  // 3. TI CC1101 Manual Power-On Reset sequence (datasheet section 19.1.2)
  digitalWrite(PIN_CC1101_SCK, HIGH);
  digitalWrite(PIN_CC1101_MOSI, LOW);
  digitalWrite(PIN_CC1101_CSN, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_CC1101_CSN, HIGH);
  delayMicroseconds(45);
  digitalWrite(PIN_CC1101_SCK, LOW);

  digitalWrite(PIN_CC1101_CSN, LOW);
  _hwDiag.chip_rdy_seen = waitMisoLow(5000UL);
  _hwDiag.miso_cs_low = static_cast<uint8_t>(digitalRead(PIN_CC1101_MISO));

  // Issue SRES strobe (0x30) and wait for CHIP_RDYn low
  (void)bbTransferByte(0x30U);
  (void)waitMisoLow(5000UL);
  digitalWrite(PIN_CC1101_CSN, HIGH);
  delay(1);

  // 4. Bit-bang read PARTNUM (0x30 | READ | BURST = 0xF0) and VERSION (0xF1)
  digitalWrite(PIN_CC1101_CSN, LOW);
  (void)waitMisoLow(1000UL);
  _hwDiag.bb_status = bbTransferByte(0xF0U);
  _hwDiag.bb_partnum = bbTransferByte(0x00U);
  digitalWrite(PIN_CC1101_CSN, HIGH);
  delayMicroseconds(20);

  digitalWrite(PIN_CC1101_CSN, LOW);
  (void)waitMisoLow(1000UL);
  (void)bbTransferByte(0xF1U);
  _hwDiag.bb_version = bbTransferByte(0x00U);
  digitalWrite(PIN_CC1101_CSN, HIGH);

  // Restore MISO without internal pull before enabling hardware SPI2
  pinMode(PIN_CC1101_MISO, INPUT);
}

bool RfReceiver::begin() {
  return beginInternal(false);
}

bool RfReceiver::beginInternal(bool runPinMatrixScan) {
  initTimer1Micros();

  probeAndResetCc1101(runPinMatrixScan);

  // Explicitly configure GPIO directions for nRF52832 legacy SPI2 peripheral
  pinMode(PIN_CC1101_SCK, OUTPUT);
  digitalWrite(PIN_CC1101_SCK, LOW);
  pinMode(PIN_CC1101_MOSI, OUTPUT);
  digitalWrite(PIN_CC1101_MOSI, LOW);
  pinMode(PIN_CC1101_MISO, INPUT);
  pinMode(PIN_CC1101_CSN, OUTPUT);
  digitalWrite(PIN_CC1101_CSN, HIGH);
  pinMode(PIN_CC1101_GDO0, INPUT);

  SPI.setPins(PIN_CC1101_MISO, PIN_CC1101_SCK, PIN_CC1101_MOSI);
  SPI.begin();

  _currentFreqMhz = FREQ_BAND_315_MHZ;
  _currentMod = RfModulation::FSK;
  _hopStep = 0;

  int16_t state = g_radio.begin(
      _currentFreqMhz,
      CC1101_FSK_BITRATE_KBPS,
      CC1101_FSK_FREQ_DEV_KHZ,
      CC1101_RX_BW_KHZ,
      10,
      16);

  _lastError = state;
  _chipVersion = g_radio.getChipVersion();
  _hwDiag.hw_version = _chipVersion;

  if (state != RADIOLIB_ERR_NONE) {
    _radioOk = false;
    return false;
  }

  // Enter asynchronous direct RX mode so raw demodulated pulses appear on GDO0
  state = g_radio.receiveDirectAsync();
  _lastError = state;
  if (state != RADIOLIB_ERR_NONE) {
    _radioOk = false;
    return false;
  }

  _radioOk = true;
  clearPulses();
  s_lastEdgeUs = readTimer1Micros();
  s_captureArmed = true;
  attachInterrupt(PIN_CC1101_GDO0, onGdo0Edge, CHANGE);

  _lastHopMs = millis();
  _deferringHop = false;
  return true;
}

bool RfReceiver::applyChannel(float freqMhz, RfModulation mod) {
  if (!_radioOk) {
    return false;
  }

  s_captureArmed = false;

  int16_t state = g_radio.standby();

  if (mod != _currentMod) {
    if (mod == RfModulation::OOK) {
      state |= g_radio.setOOK(true);
      state |= g_radio.setBitRate(CC1101_OOK_BITRATE_KBPS);
      state |= g_radio.setRxBandwidth(CC1101_RX_BW_KHZ);
    } else {
      state |= g_radio.setOOK(false);
      state |= g_radio.setBitRate(CC1101_FSK_BITRATE_KBPS);
      state |= g_radio.setFrequencyDeviation(CC1101_FSK_FREQ_DEV_KHZ);
      state |= g_radio.setRxBandwidth(CC1101_RX_BW_KHZ);
    }
    _currentMod = mod;
  }

  if (fabsf(freqMhz - _currentFreqMhz) > 0.01f) {
    state |= g_radio.setFrequency(freqMhz);
    _currentFreqMhz = freqMhz;
  }

  state |= g_radio.receiveDirectAsync();
  _lastError = state;

  s_lastEdgeUs = readTimer1Micros();
  s_captureArmed = true;
  return (state == RADIOLIB_ERR_NONE);
}

void RfReceiver::update(uint32_t nowMs, bool activeBurstInProgress) {
  if (!_radioOk) {
    return;
  }

  const bool singleBand = (_bandMode != RfBandMode::HOP);
  const bool singleMod = (_modMode != RfModMode::BOTH);
  if (singleBand && singleMod) {
    return;
  }

  if ((nowMs - _lastHopMs) < DEFAULT_HOP_INTERVAL_MS) {
    return;
  }

  // Defer channel retune briefly if the decoder is mid-frame
  if (activeBurstInProgress) {
    if (!_deferringHop) {
      _deferringHop = true;
      _deferStartMs = nowMs;
    }
    if ((nowMs - _deferStartMs) < HOP_DEFER_MAX_MS) {
      return;
    }
  }
  _deferringHop = false;
  _lastHopMs = nowMs;

  float targetFreq = _currentFreqMhz;
  RfModulation targetMod = _currentMod;

  if (!singleBand && !singleMod) {
    // 4-step sweep: 315 FSK -> 433.92 FSK -> 315 OOK -> 433.92 OOK
    _hopStep = (_hopStep + 1U) & 0x03U;
    targetFreq = ((_hopStep & 0x01U) == 0U) ? FREQ_BAND_315_MHZ : FREQ_BAND_433_MHZ;
    targetMod = ((_hopStep & 0x02U) == 0U) ? RfModulation::FSK : RfModulation::OOK;
  } else if (!singleBand) {
    targetFreq = (fabsf(_currentFreqMhz - FREQ_BAND_315_MHZ) < 1.0f)
                     ? FREQ_BAND_433_MHZ
                     : FREQ_BAND_315_MHZ;
    targetMod = (_modMode == RfModMode::OOK_ONLY) ? RfModulation::OOK : RfModulation::FSK;
  } else {
    targetFreq = (_bandMode == RfBandMode::BAND_315) ? FREQ_BAND_315_MHZ : FREQ_BAND_433_MHZ;
    targetMod = (_currentMod == RfModulation::FSK) ? RfModulation::OOK : RfModulation::FSK;
  }

  applyChannel(targetFreq, targetMod);
}

bool RfReceiver::popPulse(PulseEvent& out) {
  const uint16_t tail = s_ringTail;
  if (tail == s_ringHead) {
    return false;
  }
  __DMB();
  out.duration_us = s_pulseRing[tail].duration_us;
  out.level = s_pulseRing[tail].level;
  out.reserved = 0;
  __DMB();
  s_ringTail = static_cast<uint16_t>((tail + 1U) & (PULSE_RING_SIZE - 1U));
  return true;
}

void RfReceiver::clearPulses() {
  noInterrupts();
  s_ringHead = 0;
  s_ringTail = 0;
  interrupts();
}

void RfReceiver::setBandMode(RfBandMode mode) {
  _bandMode = mode;
  if (!_radioOk) {
    return;
  }

  float targetFreq = _currentFreqMhz;
  if (mode == RfBandMode::BAND_315) {
    targetFreq = FREQ_BAND_315_MHZ;
  } else if (mode == RfBandMode::BAND_433) {
    targetFreq = FREQ_BAND_433_MHZ;
  }
  applyChannel(targetFreq, _currentMod);
  _lastHopMs = millis();
}

void RfReceiver::setModMode(RfModMode mode) {
  _modMode = mode;
  if (!_radioOk) {
    return;
  }

  RfModulation targetMod = _currentMod;
  if (mode == RfModMode::FSK_ONLY) {
    targetMod = RfModulation::FSK;
  } else if (mode == RfModMode::OOK_ONLY) {
    targetMod = RfModulation::OOK;
  }
  applyChannel(_currentFreqMhz, targetMod);
  _lastHopMs = millis();
}

float RfReceiver::currentRssiDbm() {
  if (!_radioOk) {
    return -128.0f;
  }
  return g_radio.getRSSI();
}

uint32_t RfReceiver::totalPulses() const {
  return s_totalPulses;
}

uint32_t RfReceiver::droppedPulses() const {
  return s_droppedPulses;
}

void RfReceiver::sampleBandRssi(float& rssi315Dbm, float& rssi433Dbm) {
  if (!_radioOk) {
    rssi315Dbm = -128.0f;
    rssi433Dbm = -128.0f;
    return;
  }

  const float savedFreq = _currentFreqMhz;
  const RfModulation savedMod = _currentMod;

  applyChannel(FREQ_BAND_315_MHZ, RfModulation::FSK);
  delay(8);
  rssi315Dbm = g_radio.getRSSI();

  applyChannel(FREQ_BAND_433_MHZ, RfModulation::FSK);
  delay(8);
  rssi433Dbm = g_radio.getRSSI();

  applyChannel(savedFreq, savedMod);
}

RfHardwareDiag RfReceiver::runHardwareDiag() {
  const RfBandMode savedBand = _bandMode;
  const RfModMode savedModMode = _modMode;
  const float savedFreq = _currentFreqMhz;
  const RfModulation savedMod = _currentMod;

  s_captureArmed = false;
  detachInterrupt(PIN_CC1101_GDO0);

  if (beginInternal(true)) {
    _bandMode = savedBand;
    _modMode = savedModMode;
    float restoreFreq = savedFreq;
    if (savedBand == RfBandMode::BAND_315) {
      restoreFreq = FREQ_BAND_315_MHZ;
    } else if (savedBand == RfBandMode::BAND_433) {
      restoreFreq = FREQ_BAND_433_MHZ;
    }
    RfModulation restoreMod = savedMod;
    if (savedModMode == RfModMode::FSK_ONLY) {
      restoreMod = RfModulation::FSK;
    } else if (savedModMode == RfModMode::OOK_ONLY) {
      restoreMod = RfModulation::OOK;
    }
    applyChannel(restoreFreq, restoreMod);
  }

  return _hwDiag;
}

void RfReceiver::setBenchPinLevels(bool high, uint8_t& misoOut, uint8_t& gdo0Out) {
  s_captureArmed = false;
  detachInterrupt(PIN_CC1101_GDO0);
  SPI.end();
  _radioOk = false;

  const uint8_t lvl = high ? HIGH : LOW;
  pinMode(PIN_CC1101_CSN, OUTPUT);
  digitalWrite(PIN_CC1101_CSN, lvl);
  pinMode(PIN_CC1101_SCK, OUTPUT);
  digitalWrite(PIN_CC1101_SCK, lvl);
  pinMode(PIN_CC1101_MOSI, OUTPUT);
  digitalWrite(PIN_CC1101_MOSI, lvl);

  pinMode(PIN_CC1101_MISO, INPUT_PULLUP);
  pinMode(PIN_CC1101_GDO0, INPUT);
  delayMicroseconds(100);

  misoOut = static_cast<uint8_t>(digitalRead(PIN_CC1101_MISO));
  gdo0Out = static_cast<uint8_t>(digitalRead(PIN_CC1101_GDO0));
}
