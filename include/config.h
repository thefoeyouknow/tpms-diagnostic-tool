// Copyright (c) 2026 Michael Schader. All Rights Reserved.

#pragma once

#include <Arduino.h>
#include <RadioLib.h>

// =============================================================================
// Firmware Identity & Versioning
// =============================================================================
#define TPMS_FW_NAME              "TPMS-Diag"
#define TPMS_FW_VERSION           "0.1.0"
#define TPMS_BLE_DEVICE_NAME      "TPMS-Diag"
#define TPMS_SERIAL_BAUD          115200

// =============================================================================
// Master Pin Assignment (Adafruit Bluefruit nRF52832 Feather)
// =============================================================================
#define PIN_CC1101_SCK            11           // P0.11 (custom remapped SPI clock)
#define PIN_CC1101_MOSI           MOSI         // P0.13 (default hardware MOSI)
#define PIN_CC1101_MISO           MISO         // P0.14 (default hardware MISO)
#define PIN_CC1101_CSN            30           // P0.30 (custom remapped chip select)
#define PIN_CC1101_GDO0           15           // P0.15 (async pulse capture interrupt)
#define PIN_CC1101_GDO2           RADIOLIB_NC

#define PIN_LF_WAKE_PWM           27           // P0.27 -> 100R -> FQP30N06L Gate drive
#define PIN_VBAT_SENSE            31           // P0.31 (onboard 100k/100k voltage divider)

// Reserved Pins (Do Not Connect externally)
#define PIN_RESERVED_FRST         7            // Factory Reset monitoring during boot
#define PIN_RESERVED_DFU          20           // Native bootloader entry trigger

// Onboard Confirmation LED (Feather Red LED = Pin 17)
#define PIN_STATUS_LED            LED_RED
#define LED_PULSE_MS              80U

// =============================================================================
// Sub-GHz RF Configuration (CC1101)
// =============================================================================
static constexpr float FREQ_BAND_315_MHZ        = 315.00f;
static constexpr float FREQ_BAND_433_MHZ        = 433.92f;
static constexpr uint32_t DEFAULT_HOP_INTERVAL_MS = 400U;
static constexpr uint32_t HOP_DEFER_MAX_MS      = 80U;

static constexpr float CC1101_FSK_BITRATE_KBPS  = 19.2f;
static constexpr float CC1101_FSK_FREQ_DEV_KHZ  = 35.0f;
static constexpr float CC1101_RX_BW_KHZ         = 135.0f;
static constexpr float CC1101_OOK_BITRATE_KBPS  = 9.6f;

// Pulse capture ring buffer size (power of 2 for fast lock-free masking)
#define PULSE_RING_SIZE           2048U

// =============================================================================
// 125 kHz LF Wake Driver Configuration (HwPWM2 on Pin 27)
// =============================================================================
// nRF52 PWM base clock at DIV_1 is 16 MHz; 16,000,000 / 128 = 125,000 Hz exact.
#define LF_PWM_CLOCK_DIV          PWM_PRESCALER_PRESCALER_DIV_1
#define LF_PWM_COUNTERTOP         128U
#define LF_PWM_DUTY_50PCT         64U
#define LF_PWM_FREQ_HZ            125000UL

static constexpr uint32_t LF_DEFAULT_BURST_MS         = 150U;
static constexpr uint32_t LF_MAX_BURST_MS             = 500U;
static constexpr uint32_t LF_MIN_COOLDOWN_MS          = 1000U;
static constexpr uint32_t LF_DEFAULT_AUTO_INTERVAL_MS = 5000U;
static constexpr uint32_t LF_MIN_AUTO_INTERVAL_MS     = 1500U;

// Pulsed preamble timings: 5x (15 ms ON / 10 ms OFF) + 100 ms CW tail
static constexpr uint8_t  LF_PULSE_COUNT              = 5U;
static constexpr uint32_t LF_PULSE_ON_MS              = 15U;
static constexpr uint32_t LF_PULSE_OFF_MS             = 10U;
static constexpr uint32_t LF_PULSE_TAIL_CW_MS         = 100U;

// =============================================================================
// Battery & Power Monitoring (SAADC on Pin 31 / A7)
// =============================================================================
static constexpr float    VBAT_DIVIDER_RATIO          = 1.403f;  // 2M + 806k Feather nRF52832 divider
static constexpr float    VBAT_ADC_REF_V              = 3.6f;    // AR_DEFAULT (0..3.6V range)
static constexpr float    VBAT_ADC_MAX_COUNTS         = 4096.0f; // 12-bit SAADC
static constexpr float    VBAT_MIN_V                  = 3.30f;  // 0% SoC cutoff
static constexpr float    VBAT_MAX_V                  = 4.20f;  // 100% SoC full charge
static constexpr float    VBAT_LOW_WARN_V             = 3.45f;  // Low battery flag
static constexpr uint32_t VBAT_SAMPLE_INTERVAL_MS     = 2000U;

// =============================================================================
// Telemetry Heartbeat
// =============================================================================
static constexpr uint32_t STATUS_HEARTBEAT_MS         = 5000U;
