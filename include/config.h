#ifndef CONFIG_H
#define CONFIG_H

// 플랫폼 독립 상수. 호스트 테스트에서도 include하므로 Arduino 헤더를 넣지 않는다.
// (핀 매크로는 Arduino 쪽 .cpp에서만 전개된다)

#include <stdint.h>

// --- 샘플링 (FIRMWARE_SPEC §2) ---
constexpr float    kFsHz          = 104.0f;   // IMU ODR = 게이트 fs (확정)
constexpr uint16_t kImuOdrHz      = 104;      // 가속도·자이로 ODR
constexpr uint16_t kGyroRangeDps  = 500;      // BLE_SPEC §3 확정
constexpr uint8_t  kAccelRangeG   = 4;        // BLE_SPEC §3 확정 (로깅 전용)
constexpr uint8_t  kImuI2cAddr    = 0x6A;
constexpr uint32_t kImuI2cHz      = 400000;   // FIFO 4샘플(48 B) 버스트 ≈ 1.3 ms

// raw LSB → 물리량 (LSM6DS3TR-C 데이터시트 감도). BLE로는 raw를 보낸다.
constexpr float    kGyroDpsPerLsb = 0.0175f;      // ±500 dps: 17.50 mdps/LSB
constexpr float    kAccelGPerLsb  = 0.000122f;    // ±4 g: 0.122 mg/LSB
static_assert(kGyroRangeDps == 500 && kAccelRangeG == 4, "감도 상수를 풀스케일에 맞춰 고칠 것");

// --- FIFO (BLE_SPEC §3) ---
constexpr int      kSamplesPerBatch = 4;          // 워터마크 = 패킷당 샘플 수
constexpr uint32_t kSamplePeriodUs  = 9615;       // 1/104 Hz 공칭값 (타임스탬프 외삽에만 사용)

// --- 햅틱 (§6) ---
constexpr uint8_t  kDrv2605Addr   = 0x5A;
constexpr uint8_t  kVibLevel      = 80;       // RTP 레벨 (1–127). Constant·Adaptive 공통
constexpr uint32_t kBrakeMs       = 100;      // OFF 후 standby 진입까지
#define DRV_EN_PIN D1                         // DRV2605L EN (부팅 시 HIGH 필요)

// --- Serial ---
constexpr uint32_t kSerialBaud    = 115200;

#endif
