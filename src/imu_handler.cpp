#include "imu_handler.h"
#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "timebase.h"

static LSM6DS3 myIMU(I2C_MODE, kImuI2cAddr);

// --- LSM6DS3TR-C 레지스터 ---
static constexpr uint8_t kRegFifoCtrl1  = 0x06;   // FTH[7:0] (워드 단위)
static constexpr uint8_t kRegFifoCtrl2  = 0x07;   // FTH[10:8]
static constexpr uint8_t kRegFifoCtrl3  = 0x08;   // DEC_FIFO_GYRO[5:3], DEC_FIFO_XL[2:0]
static constexpr uint8_t kRegFifoCtrl4  = 0x09;
static constexpr uint8_t kRegFifoCtrl5  = 0x0A;   // ODR_FIFO[6:3], FIFO_MODE[2:0]
static constexpr uint8_t kRegInt1Ctrl   = 0x0D;
static constexpr uint8_t kRegCtrl3C     = 0x12;
static constexpr uint8_t kRegFifoStatus = 0x3A;   // STATUS1..4 연속
static constexpr uint8_t kRegFifoData   = 0x3E;   // 버스트 읽기 시 0x3F 다음 0x3E로 되돌아감

static constexpr uint8_t kCtrl3BDU       = 0x40;
static constexpr uint8_t kCtrl3IfInc     = 0x04;
static constexpr uint8_t kFifoDecNone    = 0x09;               // 자이로·가속도 모두 decimation 없음
static constexpr uint8_t kFifoContinuous = (0x4 << 3) | 0x6;   // ODR_FIFO 104 Hz, continuous
static constexpr uint8_t kFifoBypass     = 0x00;               // FIFO 비움
static constexpr uint8_t kInt1Fth        = 0x08;
static constexpr uint8_t kStatus2Overrun = 0x40;

static constexpr int kWordsPerSample = 6;   // FIFO 패턴: Gx Gy Gz XLx XLy XLz
static constexpr int kBatchWords     = kWordsPerSample * kSamplesPerBatch;
static constexpr int kBatchBytes     = kBatchWords * 2;   // 48 B (Wire 버퍼 64 B 이내)
static_assert(kBatchBytes <= 64, "Wire RX 버퍼(64 B)를 넘는다");

// INT1 무응답 시 FIFO 상태를 직접 확인하는 간격 (정상이면 쓰이지 않음)
static constexpr uint32_t kFallbackPollUs = 50000;

static volatile uint32_t irq_cnt = 0;
static volatile uint32_t irq_t = 0;

static uint32_t seen_irq = 0;
static uint32_t seq = 0;
static uint32_t last_t = 0;          // 마지막으로 내보낸 묶음의 t_us
static bool     have_last_t = false;
static bool     more = false;        // 직전 확인에서 묶음이 하나 더 남아 있었음
static bool     discont = false;
static uint32_t last_check_us = 0;
static ImuStats stats = {};

static void onInt1() {
    irq_t = timebase_us();
    irq_cnt = irq_cnt + 1;
}

static bool wr(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(kImuI2cAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool rd(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(kImuI2cAddr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(kImuI2cAddr, n) != n) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
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
    Wire.setClock(kImuI2cHz);   // begin()이 Wire.begin()으로 100 kHz로 되돌리므로 그 뒤에

    uint8_t ctrl3 = 0;
    if (!rd(kRegCtrl3C, &ctrl3, 1)) return false;
    // BDU: 상·하위 바이트가 다른 샘플에서 오지 않게. IF_INC: 버스트 읽기
    if (!wr(kRegCtrl3C, ctrl3 | kCtrl3BDU | kCtrl3IfInc)) return false;

    bool ok = wr(kRegFifoCtrl5, kFifoBypass) &&
              wr(kRegFifoCtrl1, (uint8_t)(kBatchWords & 0xFF)) &&
              wr(kRegFifoCtrl2, (uint8_t)((kBatchWords >> 8) & 0x07)) &&
              wr(kRegFifoCtrl3, kFifoDecNone) &&
              wr(kRegFifoCtrl4, 0x00) &&
              wr(kRegInt1Ctrl, kInt1Fth);
    if (!ok) return false;

    static bool attached = false;
    if (!attached) {
        pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
        attachInterrupt(PIN_LSM6DS3TR_C_INT1, onInt1, RISING);
        attached = true;
    }

    if (!wr(kRegFifoCtrl5, kFifoContinuous)) return false;
    uint8_t back = 0;
    if (!rd(kRegFifoCtrl5, &back, 1) || back != kFifoContinuous) return false;

    // 이전 상태 버림. seq는 이어간다 (재초기화 동안의 공백은 시각으로 알 수 있음)
    seen_irq = irq_cnt;
    have_last_t = false;
    more = false;
    discont = true;
    last_check_us = timebase_us();
    return true;
}

uint32_t imuNextSeq() { return seq; }
const ImuStats& imuStats() { return stats; }

// overrun: FIFO가 가득 차 오래된 샘플이 덮어써졌다. 남은 내용은 시각을 알 수 없으므로
// 전부 버리고, 마지막 묶음 이후 흐른 시간만큼 seq를 건너뛴다.
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
    const int pattern = ((st[3] & 0x03) << 8) | st[2];   // 다음에 읽을 워드의 위치 (0 = Gx)
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
        if (fresh) seen_irq = c;   // 묶음 없는 INT1 (재정렬 직후 등)
        return false;
    }

    // 이 묶음의 마지막 샘플 시각.
    // INT1은 FIFO가 3→4샘플이 되는 순간 오르고, 그때 FIFO의 가장 오래된 4개가 다음에 읽을
    // 묶음이다. 그래서 확인하지 않은 INT1 시각은 곧 이번 묶음의 시각이다. 여러 묶음이 쌓여
    // INT1이 계속 HIGH면 새 에지가 없으므로 직전 묶음 + 4T로 외삽한다.
    const uint32_t extrap = last_t + kSamplesPerBatch * kSamplePeriodUs;
    uint32_t t_batch;
    bool exact = false;
    if (fresh) {
        seen_irq = c;
        // ISR이 늦게 돌아 직전 묶음의 INT1을 지금 본 경우는 버린다
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
            // 기준 시각 없음: 가장 최근 샘플 ≈ now 로 보고 거슬러 계산
            const int samples = words / kWordsPerSample;
            t_batch = now - (uint32_t)(samples - kSamplesPerBatch) * kSamplePeriodUs;
        }
    }

    uint8_t b[kBatchBytes];
    if (!rd(kRegFifoData, b, sizeof(b))) {
        ++stats.i2c_err;
        more = false;
        discont = true;   // 일부만 읽혔을 수 있다 → 다음 확인에서 패턴으로 재정렬
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
