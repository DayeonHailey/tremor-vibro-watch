#include "haptic_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// --- DRV2605L 레지스터 맵 ---
#define REG_MODE            0x01
#define REG_RTPIN           0x02
#define REG_LIBSEL          0x03
#define REG_OD_CLAMP        0x17   // 구동 전압 상한
#define REG_FEEDBACK        0x1A   // bit7 N_ERM_LRA
#define REG_CONTROL3        0x1D
#define REG_OL_LRA_PERIOD   0x20   // open-loop LRA 구동 주기 (0x22는 읽기 전용이라 쓰지 않음)

#define MODE_RTP            0x05
#define MODE_STANDBY_RTP    0x45   // bit6 STANDBY + RTP 모드 유지

// OL_LRA_PERIOD: 주기 = 값 × 98.46 us
#define CALC_LRA_PERIOD(hz) (uint8_t)(1.0f / ((hz) * 0.00009846f))

// 구동 전압 상한 1.8 V. OD_CLAMP 환산식 21.96 mV/LSB (§13-4, 데이터시트로 최종 확인 필요)
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
    delay(10);    // IC 부팅 대기

    writeRegister8(REG_MODE, 0x00);                  // standby 해제
    writeRegister8(REG_FEEDBACK, 0x80 | 0x06);       // LRA 모드
    writeRegister8(REG_LIBSEL, 0x06);                // LRA 라이브러리
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
        writeRegister8(REG_RTPIN, 0x00);   // 제동 시작
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
