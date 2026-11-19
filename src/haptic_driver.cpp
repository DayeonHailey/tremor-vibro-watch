#include "haptic_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// --- DRV2605L register map ---
#define REG_MODE            0x01
#define REG_RTPIN           0x02
#define REG_LIBSEL          0x03
#define REG_OD_CLAMP        0x17   // drive voltage limit
#define REG_FEEDBACK        0x1A   // bit7 N_ERM_LRA
#define REG_CONTROL3        0x1D
#define REG_OL_LRA_PERIOD   0x20   // open-loop LRA drive period (0x22 is read-only, so not written)

#define MODE_RTP            0x05
#define MODE_STANDBY_RTP    0x45   // bit6 STANDBY + keep RTP mode

// OL_LRA_PERIOD: period = value × 98.46 us
#define CALC_LRA_PERIOD(hz) (uint8_t)(1.0f / ((hz) * 0.00009846f))

// Drive voltage limit 1.8 V. OD_CLAMP conversion 21.96 mV/LSB (§13-4, to be confirmed against the datasheet)
// → 1800 / 21.96 = 81.97 ≈ 82 (0x52, 1.80 V)
constexpr float   kVoltageLimitV = 1.8f;
constexpr uint8_t kOdClamp = (uint8_t)(kVoltageLimitV * 1000.0f / 21.96f + 0.5f);

enum class DrvState : uint8_t { STANDBY, ACTIVE, BRAKING };

static DrvState state = DrvState::STANDBY;
static uint32_t brake_start_ms = 0;

static void writeRegister8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(kDrv2605Addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void Haptic_Init() {
    pinMode(DRV_EN_PIN, OUTPUT);
    digitalWrite(DRV_EN_PIN, HIGH);
    delay(10);

    Wire.begin();
    delay(10);    // wait for IC boot

    writeRegister8(REG_MODE, 0x00);                  // exit standby
    writeRegister8(REG_FEEDBACK, 0x80 | 0x06);       // LRA mode
    writeRegister8(REG_LIBSEL, 0x06);                // LRA library
    writeRegister8(REG_CONTROL3, 0x01);              // LRA_OPEN_LOOP, RTP signed
    writeRegister8(REG_OD_CLAMP, kOdClamp);
    writeRegister8(REG_OL_LRA_PERIOD, CALC_LRA_PERIOD(170));

    writeRegister8(REG_MODE, MODE_RTP);
    writeRegister8(REG_RTPIN, 0x00);
    writeRegister8(REG_MODE, MODE_STANDBY_RTP);
    state = DrvState::STANDBY;
}

void Haptic_Drive(bool on) {
    if (on) {
        if (state == DrvState::ACTIVE) return;
        if (state == DrvState::STANDBY) writeRegister8(REG_MODE, MODE_RTP);
        writeRegister8(REG_RTPIN, kVibLevel);
        state = DrvState::ACTIVE;
        return;
    }

    if (state == DrvState::ACTIVE) {
        writeRegister8(REG_RTPIN, 0x00);   // start braking
        brake_start_ms = millis();
        state = DrvState::BRAKING;
    } else if (state == DrvState::BRAKING && millis() - brake_start_ms >= kBrakeMs) {
        writeRegister8(REG_MODE, MODE_STANDBY_RTP);
        state = DrvState::STANDBY;
    }
}

bool Haptic_IsActive() {
    return state == DrvState::ACTIVE;
}
