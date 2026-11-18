#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// LSM6DS3TR-C FIFO 기반 샘플 획득 (BLE_SPEC §3, S1).
//   FIFO continuous, 자이로·가속도 104 Hz, 워터마크 4샘플 → INT1 (P0.11, PIN_LSM6DS3TR_C_INT1)
//   INT1 상승 에지 ISR에서 timebase_us()를 기록 → 묶음의 마지막(4번째) 샘플 시각
//   seq는 부팅 후 샘플마다 +1. overrun으로 잃은 샘플도 seq를 소비한다.

#include <stdint.h>
#include "config.h"

struct ImuRaw {
    int16_t g[3];   // 자이로 x, y, z (raw LSB)
    int16_t a[3];   // 가속도 x, y, z (raw LSB)
};

struct ImuBatch {
    ImuRaw   s[kSamplesPerBatch];
    uint32_t seq0;        // s[0]의 seq
    uint32_t t_us;        // s[kSamplesPerBatch-1]의 시각 (timebase_us)
    bool     t_exact;     // false: INT1 시각이 아니라 직전 묶음에서 외삽한 값
    bool     discontinuity;  // 직전 묶음과 seq가 이어지지 않음 (overrun 후 첫 묶음)
};

struct ImuStats {
    uint16_t fifo_overrun;   // overrun 발생 횟수
    uint32_t lost_samples;   // overrun으로 건너뛴 seq 합계 (추정)
    uint16_t i2c_err;        // I2C 실패 (짧은 읽기 포함)
    uint16_t realign;        // FIFO 패턴이 어긋나 버린 횟수
    uint16_t ts_extrap;      // INT1 시각 없이 외삽한 묶음 수
    uint16_t ts_stale_irq;   // 이미 처리한 묶음의 INT1이라 버린 횟수
    uint32_t irq_count;      // INT1 ISR 횟수
    uint16_t gyro_clip;      // 자이로 축 하나라도 풀스케일 근처인 샘플 수
};

bool imuInit();                 // 실패 시 false (재호출 가능)
bool imuPoll(ImuBatch* out);    // 묶음 하나를 꺼냈으면 true. loop에서 false가 될 때까지 반복 호출
uint32_t imuNextSeq();          // 다음 샘플에 붙을 seq
const ImuStats& imuStats();

inline float imuGyroDps(int16_t raw) { return raw * kGyroDpsPerLsb; }
inline float imuAccelG(int16_t raw) { return raw * kAccelGPerLsb; }
inline bool  imuIsClipped(int16_t raw) { return raw >= 32000 || raw <= -32000; }

#endif
