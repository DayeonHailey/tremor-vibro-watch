#include "imu_handler.h"
#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "timebase.h"

static LSM6DS3 myIMU(I2C_MODE, kImuI2cAddr);

// On the XIAO nRF52840 Sense the IMU is on the internal I2C bus (Wire1, P0.07/P0.27).
// The library also uses Wire1 on this board via #define Wire Wire1. The DRV2605L is on the external Wire.
static TwoWire& imuWire = Wire1;

// --- LSM6DS3TR-C registers ---
static constexpr uint8_t kRegFifoCtrl1  = 0x06;   // FTH[7:0] (in words)
static constexpr uint8_t kRegFifoCtrl2  = 0x07;   // FTH[10:8]
static constexpr uint8_t kRegFifoCtrl3  = 0x08;   // DEC_FIFO_GYRO[5:3], DEC_FIFO_XL[2:0]
static constexpr uint8_t kRegFifoCtrl4  = 0x09;
static constexpr uint8_t kRegFifoCtrl5  = 0x0A;   // ODR_FIFO[6:3], FIFO_MODE[2:0]
static constexpr uint8_t kRegInt1Ctrl   = 0x0D;
static constexpr uint8_t kRegCtrl3C     = 0x12;
static constexpr uint8_t kRegFifoStatus = 0x3A;   // STATUS1..4, contiguous
static constexpr uint8_t kRegFifoData   = 0x3E;   // burst reads wrap from 0x3F back to 0x3E

static constexpr uint8_t kCtrl3BDU       = 0x40;
static constexpr uint8_t kCtrl3IfInc     = 0x04;
static constexpr uint8_t kFifoDecNone    = 0x09;               // no decimation for gyro or accel
static constexpr uint8_t kFifoContinuous = (0x4 << 3) | 0x6;   // ODR_FIFO 104 Hz, continuous
static constexpr uint8_t kFifoBypass     = 0x00;               // empties the FIFO
static constexpr uint8_t kInt1Fth        = 0x08;
static constexpr uint8_t kStatus2Overrun = 0x40;

static constexpr int kWordsPerSample = 6;   // FIFO pattern: Gx Gy Gz XLx XLy XLz
static constexpr int kBatchWords     = kWordsPerSample * kSamplesPerBatch;
static constexpr int kBatchBytes     = kBatchWords * 2;   // 48 B (within the 64 B Wire buffer)
static_assert(kBatchBytes <= 64, "exceeds Wire RX buffer (64 B)");

// Interval for checking FIFO status directly if INT1 is silent (unused in normal operation)
static constexpr uint32_t kFallbackPollUs = 50000;

static volatile uint32_t irq_cnt = 0;
static volatile uint32_t irq_t = 0;

static uint32_t seen_irq = 0;
static uint32_t seq = 0;
static uint32_t last_t = 0;          // t_us of the last emitted batch
static bool     have_last_t = false;
static bool     more = false;        // the last check showed at least one more batch
static bool     discont = false;
static uint32_t last_check_us = 0;
static ImuStats stats = {};

static void onInt1() {
    irq_t = timebase_us();
    irq_cnt = irq_cnt + 1;
}

static bool wr(uint8_t reg, uint8_t val) {
    imuWire.beginTransmission(kImuI2cAddr);
    imuWire.write(reg);
    imuWire.write(val);
    return imuWire.endTransmission() == 0;
}

static bool rd(uint8_t reg, uint8_t* buf, uint8_t n) {
    imuWire.beginTransmission(kImuI2cAddr);
    imuWire.write(reg);
    if (imuWire.endTransmission() != 0) return false;
    if (imuWire.requestFrom(kImuI2cAddr, n) != n) {
        while (imuWire.available()) imuWire.read();
        return false;
    }
    for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)imuWire.read();
    return true;
}

static bool fifoRestart() {
    return wr(kRegFifoCtrl5, kFifoBypass) && wr(kRegFifoCtrl5, kFifoContinuous);
}

bool imuInit() {
    myIMU.settings.gyroRange       = kGyroRangeDps;
    myIMU.settings.gyroSampleRate  = kImuOdrHz;
    myIMU.settings.accelRange      = kAccelRangeG;
    myIMU.settings.accelSampleRate = kImuOdrHz;
    myIMU.settings.accelBandWidth  = 50;
    if (myIMU.begin() != 0) return false;
    imuWire.setClock(kImuI2cHz);   // after begin(), since Wire1.begin() resets it to 100 kHz

    uint8_t ctrl3 = 0;
    if (!rd(kRegCtrl3C, &ctrl3, 1)) return false;
    // BDU: high/low bytes always from the same sample. IF_INC: burst reads
    if (!wr(kRegCtrl3C, ctrl3 | kCtrl3BDU | kCtrl3IfInc)) return false;

    if (!(wr(kRegFifoCtrl5, kFifoBypass) &&
          wr(kRegFifoCtrl1, (uint8_t)(kBatchWords & 0xFF)) &&
          wr(kRegFifoCtrl2, (uint8_t)((kBatchWords >> 8) & 0x07)) &&
          wr(kRegFifoCtrl3, kFifoDecNone) &&
          wr(kRegFifoCtrl4, 0x00) &&
          wr(kRegInt1Ctrl, kInt1Fth)))
        return false;

    static bool attached = false;
    if (!attached) {
        pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
        attachInterrupt(PIN_LSM6DS3TR_C_INT1, onInt1, RISING);
        attached = true;
    }

    if (!wr(kRegFifoCtrl5, kFifoContinuous)) return false;
    uint8_t back = 0;
    if (!rd(kRegFifoCtrl5, &back, 1) || back != kFifoContinuous) return false;

    // Discard previous state. seq continues (the gap during re-init is visible from timestamps)
    seen_irq = irq_cnt;
    have_last_t = false;
    more = false;
    discont = true;
    last_check_us = timebase_us();
    return true;
}

uint32_t imuNextSeq() { return seq; }
const ImuStats& imuStats() { return stats; }

// overrun: the FIFO filled up and old samples were overwritten. The remaining contents have unknown
// timestamps, so discard them all and skip seq by the time elapsed since the last batch.
static void handleOverrun(uint32_t now) {
    ++stats.fifo_overrun;
    if (have_last_t) {
        const uint32_t lost = (now - last_t + kSamplePeriodUs / 2) / kSamplePeriodUs;
        seq += lost;
        stats.lost_samples += lost;
    }
    if (!fifoRestart()) ++stats.i2c_err;
    have_last_t = false;
    more = false;
    discont = true;
}

static bool discardWords(int n) {
    uint8_t tmp[2];
    for (int i = 0; i < n; ++i) {
        if (!rd(kRegFifoData, tmp, 2)) return false;
    }
    return true;
}

bool imuPoll(ImuBatch* out) {
    uint32_t c, t;
    do {
        c = irq_cnt;
        t = irq_t;
    } while (c != irq_cnt);
    const bool fresh = (c != seen_irq);
    const uint32_t now = timebase_us();

    if (!fresh && !more && (now - last_check_us) < kFallbackPollUs) return false;
    last_check_us = now;

    uint8_t st[4];
    if (!rd(kRegFifoStatus, st, sizeof(st))) {
        ++stats.i2c_err;
        return false;
    }
    if (st[1] & kStatus2Overrun) {
        handleOverrun(now);
        seen_irq = c;
        return false;
    }
    int words = ((st[1] & 0x0F) << 8) | st[0];
    const int pattern = ((st[3] & 0x03) << 8) | st[2];   // position of the next word to read (0 = Gx)
    if (pattern != 0) {
        const int skip = (kWordsPerSample - pattern % kWordsPerSample) % kWordsPerSample;
        ++stats.realign;
        if (skip > words || !discardWords(skip)) {
            ++stats.i2c_err;
            more = false;
            return false;
        }
        words -= skip;
        discont = true;
    }
    if (words < kBatchWords) {
        more = false;
        if (fresh) seen_irq = c;   // INT1 with no batch (e.g. right after realignment)
        return false;
    }

    // Time of the last sample in this batch.
    // INT1 rises when the FIFO goes from 3 to 4 samples; at that moment the 4 oldest samples are the
    // next batch to read, so an unconsumed INT1 time is this batch's time. When several batches pile up,
    // INT1 stays HIGH with no new edge, so extrapolate as previous batch + 4T.
    const uint32_t extrap = last_t + kSamplesPerBatch * kSamplePeriodUs;
    uint32_t t_batch;
    bool exact = false;
    if (fresh) {
        seen_irq = c;
        // Discard an INT1 for the previous batch that we only see now because the ISR ran late
        if (have_last_t && (int32_t)(t - extrap) < -(int32_t)(2 * kSamplePeriodUs)) {
            ++stats.ts_stale_irq;
        } else {
            t_batch = t;
            exact = true;
        }
    }
    if (!exact) {
        ++stats.ts_extrap;
        if (have_last_t) {
            t_batch = extrap;
        } else {
            // No reference time: assume the newest sample ≈ now and count backwards
            const int samples = words / kWordsPerSample;
            t_batch = now - (uint32_t)(samples - kSamplesPerBatch) * kSamplePeriodUs;
        }
    }

    uint8_t b[kBatchBytes];
    if (!rd(kRegFifoData, b, sizeof(b))) {
        ++stats.i2c_err;
        more = false;
        discont = true;   // may have been partially read → realign by pattern on the next check
        return false;
    }
    for (int i = 0; i < kSamplesPerBatch; ++i) {
        const uint8_t* p = b + i * kWordsPerSample * 2;
        ImuRaw& s = out->s[i];
        bool clip = false;
        for (int k = 0; k < 3; ++k) {
            s.g[k] = (int16_t)(p[2 * k] | (p[2 * k + 1] << 8));
            s.a[k] = (int16_t)(p[6 + 2 * k] | (p[6 + 2 * k + 1] << 8));
            clip = clip || imuIsClipped(s.g[k]);
        }
        if (clip) ++stats.gyro_clip;
    }
    out->seq0 = seq;
    out->t_us = t_batch;
    out->t_exact = exact;
    out->discontinuity = discont;
    seq += kSamplesPerBatch;
    last_t = t_batch;
    have_last_t = true;
    discont = false;
    more = (words - kBatchWords) >= kBatchWords;
    stats.irq_count = c;
    return true;
}
