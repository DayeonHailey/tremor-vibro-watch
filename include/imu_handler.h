#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// LSM6DS3TR-C FIFO-based sample acquisition (BLE_SPEC §3, S1).
//   FIFO continuous, gyro/accel 104 Hz, watermark 4 samples → INT1 (P0.11, PIN_LSM6DS3TR_C_INT1)
//   INT1 rising-edge ISR records timebase_us() → time of the last (4th) sample in the batch
//   seq increments by 1 per sample since boot. Samples lost to overrun still consume seq.

#include <stdint.h>
#include "config.h"

struct ImuRaw {
    int16_t g[3];   // gyro x, y, z (raw LSB)
    int16_t a[3];   // accel x, y, z (raw LSB)
};

struct ImuBatch {
    ImuRaw   s[kSamplesPerBatch];
    uint32_t seq0;        // seq of s[0]
    uint32_t t_us;        // time of s[kSamplesPerBatch-1] (timebase_us)
    bool     t_exact;     // false: extrapolated from the previous batch, not an INT1 time
    bool     discontinuity;  // seq does not continue from the previous batch (first batch after overrun)
};

struct ImuStats {
    uint16_t fifo_overrun;   // number of overruns
    uint32_t lost_samples;   // total seq skipped due to overrun (estimated)
    uint16_t i2c_err;        // I2C failures (including short reads)
    uint16_t realign;        // times the FIFO pattern was misaligned and words discarded
    uint16_t ts_extrap;      // batches timestamped by extrapolation (no INT1 time)
    uint16_t ts_stale_irq;   // INT1 times discarded because they belonged to an already-processed batch
    uint32_t irq_count;      // INT1 ISR count
    uint16_t gyro_clip;      // samples with any gyro axis near full scale
};

bool imuInit();                 // false on failure (may be called again)
bool imuPoll(ImuBatch* out);    // true if a batch was read. Call repeatedly from loop until false
uint32_t imuNextSeq();          // seq for the next sample
const ImuStats& imuStats();

inline float imuGyroDps(int16_t raw) { return raw * kGyroDpsPerLsb; }
inline float imuAccelG(int16_t raw) { return raw * kAccelGPerLsb; }
inline bool  imuIsClipped(int16_t raw) { return raw >= 32000 || raw <= -32000; }

#endif
