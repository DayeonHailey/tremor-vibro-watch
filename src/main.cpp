#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "imu_handler.h"
#include "timebase.h"
#include "tremor_gate.h"
#include "haptic_driver.h"
#include "device_state.h"
#include "config_store.h"
#include "icd.h"
#include "ble_link.h"

// Serial commands (line based; replies start with '#' to separate them from CSV):
// Each command calls the same Device::cmd* function as the BLE Ctrl path; the reply shows the RespStatus.
//   m<0|2|3>        mode (NO_VIB / CONSTANT / ADAPTIVE)
//   p<field>=<val>  GateParams field (e.g. pf0=5.2, ptheta_on=3.1). Resets the gate on success
//   r               reset gate
//   t<id>           start trial <id> (1–255); t0 ends the current trial. m and p are rejected during a trial
//   v<ms>           test pulse (100–2000 ms, not during a trial)
//   ?               print parameters and CSV header
//   c               start cycle profiling (prints per-stage min/avg/max after 10k samples, §8)
//   s               print IMU/FIFO/BLE counters (seq, overrun, I2C errors, MTU, ...)
//
// t_us in the CSV appears only on the last sample row of each FIFO batch (same meaning as the BLE Data packet, BLE_SPEC §3).

static Device dev;
static ConfigStore store;

// Error code (BLE_SPEC §7 err_code). Init failures do not halt; retried every 1 s.
static constexpr uint8_t kErrNone = icd::ERR_NONE;
static constexpr uint8_t kErrImuInit = icd::ERR_IMU_INIT;
static uint8_t err_code = kErrNone;
static uint64_t device_id = 0;

// MTU exchange completes shortly after connecting, so MTU_LOW is reported only if still low after this time
static constexpr uint32_t kMtuGraceMs = 2000;
static bool was_connected = false;
static uint32_t connected_ms = 0;
static constexpr uint32_t kImuRetryMs = 1000;
static uint32_t imu_retry_ms = 0;

static const char kCsvHeader[] = "seq,t_us,ax,ay,az,gx,gy,gz,bits,mode,A,C,r,D,f_acf,clip";

// ---------------------------------------------------------------------------
// Cycle profiling (DWT CYCCNT)
// ---------------------------------------------------------------------------
static constexpr uint32_t kProfileSamples = 10000;
static constexpr int kProfSlots = TremorGate::kNumStages + 1;   // per stage + total

struct ProfileStat {
    uint32_t min, max;
    uint64_t sum;
};
static ProfileStat prof[kProfSlots];
static ProfileStat prof_batch;   // FIFO check/read + 4-sample processing (gate, haptics), excluding CSV
static uint32_t prof_batch_n = 0;
static uint32_t prof_left = 0;
static uint16_t prof_overrun0 = 0;

static uint32_t dwtCycles() { return DWT->CYCCNT; }

static void profileStart() {
    // CYCCNT is not reset to 0 (only differences are used)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (ProfileStat& s : prof) s = ProfileStat{UINT32_MAX, 0, 0};
    prof_batch = ProfileStat{UINT32_MAX, 0, 0};
    prof_overrun0 = imuStats().fifo_overrun;
    prof_batch_n = 0;
    prof_left = kProfileSamples;   // multiple of 4 → ends on a batch boundary
    dev.gate().setCycleCounter(dwtCycles);
}

static void profileAccumulate() {
    const uint32_t* c = dev.gate().stageCycles();
    uint32_t total = 0;
    for (int i = 0; i < kProfSlots; ++i) {
        uint32_t v = (i < TremorGate::kNumStages) ? c[i] : total;
        if (i < TremorGate::kNumStages) total += v;
        ProfileStat& s = prof[i];
        if (v < s.min) s.min = v;
        if (v > s.max) s.max = v;
        s.sum += v;
    }
    --prof_left;
}

// Called after each batch. Prints the result once enough samples have been collected.
static void profileBatch(uint32_t cycles) {
    ProfileStat& s = prof_batch;
    if (cycles < s.min) s.min = cycles;
    if (cycles > s.max) s.max = cycles;
    s.sum += cycles;
    ++prof_batch_n;
    if (prof_left > 0) return;

    dev.gate().setCycleCounter(nullptr);
    static const char* const names[kProfSlots] = {"pre", "s1", "s2", "s3", "logic", "total"};
    const float period_cyc = (float)SystemCoreClock / kFsHz;
    Serial.print("# cyc stage,min,avg,max (clk ");
    Serial.print(SystemCoreClock);
    Serial.println(" Hz)");
    for (int i = 0; i < kProfSlots; ++i) {
        Serial.print("# cyc ");
        Serial.print(names[i]);
        Serial.print(',');
        Serial.print(prof[i].min);
        Serial.print(',');
        Serial.print((uint32_t)(prof[i].sum / kProfileSamples));
        Serial.print(',');
        Serial.println(prof[i].max);
    }
    Serial.print("# cyc total max = ");
    Serial.print(100.0f * prof[kProfSlots - 1].max / period_cyc, 3);
    Serial.println(" % of sample period (gate only)");
    Serial.print("# cyc batch,");
    Serial.print(prof_batch.min);
    Serial.print(',');
    Serial.print((uint32_t)(prof_batch.sum / prof_batch_n));
    Serial.print(',');
    Serial.println(prof_batch.max);
    Serial.print("# T10b batch max = ");
    Serial.print(100.0f * prof_batch.max / (period_cyc * kSamplesPerBatch), 3);
    Serial.print(" % of batch period (pass < 5%), fifo_overrun during run = ");
    Serial.print(imuStats().fifo_overrun - prof_overrun0);
    Serial.println(" (pass 0)");
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
static char* appendU32(char* p, uint32_t v) {
    char tmp[10];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (n) *p++ = tmp[--n];
    return p;
}

static char* appendFloat(char* p, float f, int decimals) {
    if (isnan(f)) {
        memcpy(p, "nan", 3);
        return p + 3;
    }
    static const uint32_t kScale[] = {1, 10, 100, 1000, 10000};
    const uint32_t scale = kScale[decimals];
    if (f < 0.0f) {
        *p++ = '-';
        f = -f;
    }
    if (f > 2.0e9f / scale) f = 2.0e9f / scale;
    uint32_t v = (uint32_t)lroundf(f * scale);
    p = appendU32(p, v / scale);
    if (decimals > 0) {
        *p++ = '.';
        uint32_t frac = v % scale;
        for (uint32_t d = scale / 10; d > 0; d /= 10) {
            *p++ = (char)('0' + frac / d);
            frac %= d;
        }
    }
    return p;
}

// has_t: only the last sample of a batch carries t_us (others are empty)
static void printCsv(uint32_t seq, bool has_t, uint32_t t_us, const ImuRaw& s, const GateOutput& g,
                     uint8_t bits) {
    char buf[160];
    char* p = buf;
    p = appendU32(p, seq);            *p++ = ',';
    if (has_t) p = appendU32(p, t_us);
    *p++ = ',';
    for (int k = 0; k < 3; ++k) {
        p = appendFloat(p, imuAccelG(s.a[k]), 4);
        *p++ = ',';
    }
    bool clipped = false;
    for (int k = 0; k < 3; ++k) {
        p = appendFloat(p, imuGyroDps(s.g[k]), 3);
        *p++ = ',';
        clipped = clipped || imuIsClipped(s.g[k]) || imuIsClipped(s.a[k]);
    }
    p = appendU32(p, bits);           *p++ = ',';
    p = appendU32(p, dev.mode());     *p++ = ',';
    p = appendFloat(p, g.A, 3);       *p++ = ',';
    p = appendFloat(p, g.C, 3);       *p++ = ',';
    p = appendFloat(p, g.r, 3);       *p++ = ',';
    p = appendFloat(p, g.D, 3);       *p++ = ',';
    p = appendFloat(p, g.f_acf, 2);   *p++ = ',';
    *p++ = clipped ? '1' : '0';
    *p++ = '\r';
    *p++ = '\n';
    Serial.write((const uint8_t*)buf, p - buf);
}

static void printParams() {
    const GateParams& p = dev.gate().params();
    Serial.print("# cfg_ver=");
    Serial.print(dev.cfgVer());
    Serial.print(" session_id=");
    Serial.print(dev.sessionId());
    Serial.print(" mode=");
    Serial.print(dev.mode());
    Serial.print(" state=");
    Serial.print(dev.state());
    Serial.print(" trial_id=");
    Serial.print(dev.trialId());
    Serial.print(" cfg_valid=");
    Serial.print(dev.cfgValid() ? 1 : 0);
    Serial.print(" fs=");
    Serial.println(p.fs, 1);
    for (int i = 0; i < kNumParamFields; ++i) {
        Serial.print("# ");
        Serial.print(kParamFields[i].name);
        Serial.print('=');
        Serial.println(p.*(kParamFields[i].ptr), 4);
    }
    Serial.println(kCsvHeader);
}

static const char* const kRespNames[] = {"OK", "UNKNOWN_CMD", "BAD_LEN", "OUT_OF_RANGE",
                                         "LOCKED_IN_TRIAL", "NOT_CONFIGURED", "BUSY", "FLASH_ERROR"};

static void reply(uint8_t status, const char* line) {
    Serial.print(status == icd::RS_OK ? "# ok " : "# err ");
    Serial.print(line);
    if (status != icd::RS_OK) {
        Serial.print(' ');
        Serial.print(status < sizeof(kRespNames) / sizeof(kRespNames[0]) ? kRespNames[status] : "?");
    }
    Serial.println();
}

// Parses an unsigned decimal argument; false if empty, trailing junk or > max
static bool parseUint(const char* s, uint32_t max, uint32_t* out) {
    char* end;
    const unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0' || v > max) return false;
    *out = (uint32_t)v;
    return true;
}

static void printStats() {
    const ImuStats& s = imuStats();
    Serial.print("# stats t_us=");          Serial.print(timebase_us());
    Serial.print(" seq_now=");              Serial.print(imuNextSeq());
    Serial.print(" err_code=");             Serial.print(err_code);
    Serial.print(" fifo_overrun=");         Serial.print(s.fifo_overrun);
    Serial.print(" lost_samples=");         Serial.print(s.lost_samples);
    Serial.print(" i2c_err=");              Serial.print(s.i2c_err);
    Serial.print(" realign=");              Serial.print(s.realign);
    Serial.print(" ts_extrap=");            Serial.print(s.ts_extrap);
    Serial.print(" ts_stale_irq=");         Serial.print(s.ts_stale_irq);
    Serial.print(" irq_count=");            Serial.print(s.irq_count);
    Serial.print(" gyro_clip=");            Serial.println(s.gyro_clip);
    Serial.print("# ble conn=");            Serial.print(ble::connected() ? 1 : 0);
    Serial.print(" data_sub=");             Serial.print(ble::dataSubscribed() ? 1 : 0);
    Serial.print(" mtu=");                  Serial.print(ble::mtu());
    Serial.print(" conn_interval_x125=");   Serial.print(ble::connIntervalX125());
    Serial.print(" notify_fail=");          Serial.println(ble::notifyFail());
    Serial.print("# dev state=");           Serial.print(dev.state());
    Serial.print(" flags=");                Serial.print(dev.flags());
    Serial.print(" gap=");                  Serial.print(dev.gapStart());
    Serial.print("..");                     Serial.println(dev.gapEnd());
}

// ---------------------------------------------------------------------------
// Serial command parser
// ---------------------------------------------------------------------------
static void handleLine(char* line) {
    uint32_t v = 0;
    switch (line[0]) {
        case 'm':
            reply(parseUint(line + 1, 255, &v) ? dev.cmdSetMode((uint8_t)v) : icd::RS_BAD_LEN, line);
            break;
        case 'p': {
            char* eq = strchr(line + 1, '=');
            uint8_t st = icd::RS_BAD_LEN;
            if (eq && eq != line + 1) {
                *eq = '\0';
                char* end;
                const float f = strtof(eq + 1, &end);
                if (end != eq + 1 && *end == '\0') st = dev.cmdSetParamField(line + 1, f);
                *eq = '=';
            }
            reply(st, line);
            if (st == icd::RS_OK) printParams();
            break;
        }
        case 'r':
            reply(dev.cmdGateReset(), line);
            break;
        case 't':
            if (!parseUint(line + 1, 255, &v)) reply(icd::RS_BAD_LEN, line);
            else reply(v == 0 ? dev.cmdTrialEnd(dev.trialId()) : dev.cmdTrialStart((uint8_t)v), line);
            break;
        case 'v':
            reply(parseUint(line + 1, 65535, &v) ? dev.cmdTestPulse((uint16_t)v, millis()) : icd::RS_BAD_LEN,
                  line);
            break;
        case '?':
            printParams();
            break;
        case 'c':
            profileStart();
            reply(icd::RS_OK, line);   // CSV output is paused while profiling
            break;
        case 's':
            printStats();
            break;
        default:
            reply(icd::RS_UNKNOWN_CMD, line);
            break;
    }
}

static void pollSerial() {
    static char line[48];
    static uint8_t len = 0;
    static bool overflow = false;
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (len > 0 && !overflow) {
                line[len] = '\0';
                handleLine(line);
            }
            len = 0;
            overflow = false;
        } else if (len < sizeof(line) - 1) {
            line[len++] = ch;
        } else {
            overflow = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Power LED (XIAO nRF52840 RGB LED is active-LOW)
//   USB connected (VBUS detected) → green ON
//   battery only                  → blue ON
static void powerLedInit() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    // Boot indication: red blinks 3 times
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_RED, LOW);  delay(150);
        digitalWrite(LED_RED, HIGH); delay(150);
    }
}

static void powerLedUpdate() {
    static uint32_t last_ms = 0;
    static int8_t   last_usb = -1;
    const uint32_t now = millis();
    if (now - last_ms < 500) return;
    last_ms = now;

    const bool usb = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
    if (usb == last_usb) return;
    last_usb = usb;
    digitalWrite(LED_GREEN, usb ? LOW : HIGH);
    digitalWrite(LED_BLUE,  usb ? HIGH : LOW);
}

// §9: restore the last SET_PARAMS (CFG_VALID only if the CRC matches), then an in-progress trial (§8)
static void restoreFromFlash() {
    if (!store.begin()) {
        Serial.println("# InternalFS mount failed: settings not restored, not saved");
        return;
    }
    dev.setStore(&store);

    icd::ParamsWire w;
    uint8_t ver = 0;
    if (!store.loadParams(&w, &ver)) {
        Serial.println("# /params.bin missing or invalid: CFG_VALID=0");
    } else if (!dev.restoreParams(w, ver)) {
        Serial.println("# /params.bin out of range: CFG_VALID=0");
    } else {
        Serial.print("# params restored: session_id=");
        Serial.print(w.session_id);
        Serial.print(" cfg_ver=");
        Serial.println(ver);
    }

    TrialRecord tr;
    if (!store.loadTrial(&tr)) return;
    if (dev.resumeTrial(tr)) {
        Serial.print("# REBOOTED_IN_TRIAL: resumed trial ");
        Serial.print(tr.trial_id);
        Serial.print(" mode ");
        Serial.println(tr.mode);
    } else {
        Serial.println("# /trial.bin unusable: cleared");
        store.clearTrial();
    }
}

void setup() {
    powerLedInit();
    // Do not wait for Serial so the device also runs on battery without USB
    Serial.begin(kSerialBaud);
    timebase_init();

    Haptic_Init();
    if (!imuInit()) {
        err_code = kErrImuInit;
        imu_retry_ms = millis();
        Serial.println("# IMU init failed, retrying every 1 s");
    }

    device_id = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
    icd::InfoPayload info;
    info.icd_ver = icd::kIcdVersion;
    for (int i = 0; i < 3; ++i) info.fw_ver[i] = kFwVersion[i];
    info.build_hash = BUILD_HASH;
    info.device_id = device_id;
    info.gyro_fs_dps = kGyroRangeDps;
    info.acc_fs_g = kAccelRangeG;
    info.odr_hz = kImuOdrHz;
    info.n_per_pkt = icd::kSamplesPerPkt;
    dev.setInfo(info);
    restoreFromFlash();
    ble::begin(device_id);

    Serial.println("# Tremor watch FW (spec v1.1, BLE_SPEC S3: commands + state machine)");
    printParams();
}

// ---------------------------------------------------------------------------
// BLE glue (BLE_SPEC S3). State and command logic live in Device.
// ---------------------------------------------------------------------------
static bool mtuOk() { return ble::mtu() >= icd::kMinMtu; }

static uint8_t bleErrCode() {
    if (err_code != kErrNone) return err_code;
    if (ble::connected() && !mtuOk() && millis() - connected_ms >= kMtuGraceMs) return icd::ERR_MTU_LOW;
    return icd::ERR_NONE;
}

static void sendDataPacket(const ImuBatch& b, const uint8_t* bits, const GateOutput& g_last) {
    if (!dev.shouldSendData()) return;
    icd::DataPkt p;
    p.pkt_type = icd::kPktTypeData;
    p.n = icd::kSamplesPerPkt;
    p.seq0 = b.seq0;
    p.t_us = b.t_us;
    p.state = dev.state();
    p.mode = dev.mode();
    p.cfg_ver = dev.cfgVer();
    p.trial_id = dev.trialId();
    p.flags = dev.flags();
    for (int i = 0; i < icd::kSamplesPerPkt; ++i) {
        for (int k = 0; k < 3; ++k) {
            p.s[i].g[k] = b.s[i].g[k];
            p.s[i].a[k] = b.s[i].a[k];
        }
        p.s[i].bits = bits[i];
    }
    p.feat[0] = icd::scaleSat(g_last.A, icd::kScaleA);
    p.feat[1] = icd::scaleSat(g_last.C, icd::kScaleC);
    p.feat[2] = icd::scaleSat(g_last.r, icd::kScaleR);
    p.feat[3] = icd::scaleSat(g_last.D, icd::kScaleD);
    p.feat[4] = icd::scaleSat(g_last.f_acf, icd::kScaleFacf);
    if (ble::sendData(p)) dev.onDataSent(p.seq0, p.n);
}

static void sendStatus() {
    const ImuStats& s = imuStats();
    icd::StatusPkt st = {};
    st.state = dev.state();
    st.mode = dev.mode();
    st.cfg_ver = dev.cfgVer();
    st.trial_id = dev.trialId();
    st.flags = dev.flags();
    st.err_code = bleErrCode();
    st.battery_mv = 0;    // S6
    st.log_free_kb = 0;   // S5
    st.seq_now = imuNextSeq();
    st.gap_seq_start = dev.gapStart();
    st.gap_seq_end = dev.gapEnd();
    st.fifo_overrun = s.fifo_overrun;
    st.notify_fail = ble::notifyFail();
    st.i2c_err = s.i2c_err;
    st.gyro_clip = s.gyro_clip;
    st.conn_interval_x125 = ble::connIntervalX125();
    st.mtu = ble::mtu();
    ble::sendStatus(st);
}

static void handleCtrl(const ble::CtrlMsg& m) {
    uint8_t payload[icd::kRespMaxLen - icd::kRespHeader];
    uint8_t n = 0;
    const uint8_t st = dev.dispatch(m.data, m.len, payload, &n, millis());
    ble::sendResp(m.data[1], m.data[0], st, payload, n);
}

static void serviceBle() {
    const bool c = ble::connected();
    if (c && !was_connected) connected_ms = millis();
    was_connected = c;
    dev.updateLink({c, ble::dataSubscribed(), mtuOk(), imuNextSeq()});

    ble::CtrlMsg m;
    while (ble::popCtrl(&m)) handleCtrl(m);

    // 1 Hz, and immediately after a state change (so the PC can read GAP_PENDING right after re-subscribing)
    static uint32_t last_status_ms = 0;
    if (dev.takeStatusDirty() || millis() - last_status_ms >= 1000) {
        last_status_ms = millis();
        sendStatus();
    }
}

// profiling: this batch is measured (cyc0 = CYCCNT just before imuPoll, includes FIFO read)
static void processBatch(const ImuBatch& b, bool profiling, uint32_t cyc0) {
    if (b.discontinuity) dev.gate().reset();   // samples were interrupted by overrun or re-init

    GateOutput g[kSamplesPerBatch];
    uint8_t bits[kSamplesPerBatch];
    for (int i = 0; i < kSamplesPerBatch; ++i) {
        const ImuRaw& s = b.s[i];
        g[i] = dev.gate().update(Vec3{imuGyroDps(s.g[0]), imuGyroDps(s.g[1]), imuGyroDps(s.g[2])});
        if (prof_left > 0) profileAccumulate();
        // Vibration only in TRIAL (or during a TEST_PULSE outside a trial)
        Haptic_Drive(dev.motorOn(g[i], millis()));
        bits[i] = g[i].bits;
        if (Haptic_IsActive()) bits[i] |= GateBit::VIB_ON;
    }
    if (profiling) profileBatch(dwtCycles() - cyc0);

    sendDataPacket(b, bits, g[kSamplesPerBatch - 1]);

    if (prof_left == 0) {   // CSV is off while profiling (Serial output would skew timing)
        for (int i = 0; i < kSamplesPerBatch; ++i) {
            const bool last = (i == kSamplesPerBatch - 1);
            printCsv(b.seq0 + i, last, b.t_us, b.s[i], g[i], bits[i]);
        }
    }
}

void loop() {
    powerLedUpdate();
    pollSerial();
    serviceBle();

    if (err_code == kErrImuInit) {
        Haptic_Drive(false);
        if (millis() - imu_retry_ms < kImuRetryMs) return;
        imu_retry_ms = millis();
        if (!imuInit()) return;
        err_code = kErrNone;
        Serial.println("# IMU init ok");
    }

    ImuBatch b;
    for (;;) {
        const bool profiling = prof_left > 0;
        const uint32_t cyc0 = profiling ? dwtCycles() : 0;
        if (!imuPoll(&b)) break;
        processBatch(b, profiling, cyc0);
    }
}
