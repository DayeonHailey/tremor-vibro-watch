#ifndef CONFIG_H
#define CONFIG_H

// Platform-independent constants. Host tests include this too, so no Arduino headers here.
// (pin macros are expanded only in Arduino-side .cpp files)

#include <stdint.h>

// --- Sampling (FIRMWARE_SPEC §2) ---
constexpr float    kFsHz          = 104.0f;   // IMU ODR = gate fs (fixed)
constexpr uint16_t kImuOdrHz      = 104;      // accel/gyro ODR
constexpr uint16_t kGyroRangeDps  = 500;      // fixed by BLE_SPEC §3
constexpr uint8_t  kAccelRangeG   = 4;        // fixed by BLE_SPEC §3 (logging only)
constexpr uint8_t  kImuI2cAddr    = 0x6A;
constexpr uint32_t kImuI2cHz      = 400000;   // 4-sample FIFO burst (48 B) ≈ 1.3 ms

// raw LSB → physical units (LSM6DS3TR-C datasheet sensitivity). BLE sends raw values.
constexpr float    kGyroDpsPerLsb = 0.0175f;      // ±500 dps: 17.50 mdps/LSB
constexpr float    kAccelGPerLsb  = 0.000122f;    // ±4 g: 0.122 mg/LSB
static_assert(kGyroRangeDps == 500 && kAccelRangeG == 4, "update sensitivity constants to match full scale");

// --- FIFO (BLE_SPEC §3) ---
constexpr int      kSamplesPerBatch = 4;          // watermark = samples per packet
constexpr uint32_t kSamplePeriodUs  = 9615;       // nominal 1/104 Hz (used only for timestamp extrapolation)

// --- Haptics (§6) ---
constexpr uint8_t  kDrv2605Addr   = 0x5A;
constexpr uint8_t  kVibLevel      = 80;       // RTP level (1–127), shared by Constant and Adaptive
constexpr uint32_t kBrakeMs       = 100;      // time from OFF until standby
#define DRV_EN_PIN D1                         // DRV2605L EN (must be HIGH at boot)

// --- Firmware version (GET_INFO, DIS) ---
constexpr uint8_t  kFwVersion[3]  = {0, 2, 0};   // 0.2.0 = BLE_SPEC S2
#ifndef BUILD_HASH
#define BUILD_HASH 0u                             // tools/build_hash.py injects the git hash
#endif

// --- Serial ---
constexpr uint32_t kSerialBaud    = 115200;

#endif
