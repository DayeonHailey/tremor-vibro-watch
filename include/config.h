#ifndef CONFIG_H
#define CONFIG_H

// 플랫폼 독립 상수. 호스트 테스트에서도 include하므로 Arduino 헤더를 넣지 않는다.
// (핀 매크로는 Arduino 쪽 .cpp에서만 전개된다)

#include <stdint.h>

// --- 샘플링 (FIRMWARE_SPEC §2) ---
constexpr float    kFsHz          = 104.0f;   // IMU ODR = 게이트 fs (확정)
constexpr uint16_t kImuOdrHz      = 104;      // 가속도·자이로 ODR
constexpr uint16_t kGyroRangeDps  = 500;      // PLACEHOLDER (§13-1)
constexpr uint8_t  kAccelRangeG   = 8;        // 로깅 전용
constexpr uint8_t  kImuI2cAddr    = 0x6A;

// --- 햅틱 (§6) ---
constexpr uint8_t  kDrv2605Addr   = 0x5A;
constexpr uint8_t  kVibLevel      = 80;       // RTP 레벨 (1–127). Constant·Adaptive 공통
constexpr uint32_t kBrakeMs       = 100;      // OFF 후 standby 진입까지
#define DRV_EN_PIN D1                         // DRV2605L EN (부팅 시 HIGH 필요)

// --- Serial ---
constexpr uint32_t kSerialBaud    = 115200;

#endif
