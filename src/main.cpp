#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "imu_handler.h"
#include "tremor_gate.h"
#include "haptic_controller.h"
#include "haptic_driver.h"

// Serial 명령 (줄 단위, 응답은 '#'로 시작해서 CSV와 구분):
//   m<0|2|3>        모드 (NO_VIB / CONSTANT / ADAPTIVE)
//   p<필드>=<값>     GateParams 필드 (예: pf0=5.2, ptheta_on=3.1). 성공하면 게이트 리셋
//   r               게이트 리셋
//   t<0|1>          trial 종료/시작. trial 중에는 m·p를 거부
//   ?               파라미터와 CSV 헤더 출력
//   c               사이클 계측 시작 (1만 샘플 후 stage별 min/avg/max 출력, §8)

static TremorGate gate;
static HapticController hctrl;
static uint32_t seq = 0;
static uint32_t last_sample_us = 0;
static uint8_t cfg_ver = 0;

// data-ready 폴링을 샘플 주기(9615 us) 직전까지 생략해서 I2C 버스를 비워 둔다
static constexpr uint32_t kPollGuardUs = 8500;

static const char kCsvHeader[] = "seq,t_us,ax,ay,az,gx,gy,gz,bits,mode,A,C,r,D,f_acf,clip";

// ---------------------------------------------------------------------------
// 명령 처리 (전송 수단과 무관. P3에서 BLE write 콜백이 그대로 호출한다)
// ---------------------------------------------------------------------------
bool cmd_set_mode(uint8_t v) {
    CondMode m;
    if (!CondMode_fromU8(v, &m)) return false;
    return hctrl.setMode(m);
}

bool cmd_set_param(const char* name, float value) {
    if (hctrl.trialActive()) return false;
    GateParams p = gate.params();
    if (!GateParams_set(p, name, value)) return false;
    if (!gate.setParams(p)) return false;
    ++cfg_ver;
    return true;
}

void cmd_reset_gate() {
    gate.reset();
}

void cmd_set_trial(bool active) {
    hctrl.setTrialActive(active);
}

// ---------------------------------------------------------------------------
// 사이클 계측 (DWT CYCCNT)
// ---------------------------------------------------------------------------
static constexpr uint32_t kProfileSamples = 10000;
static constexpr int kProfSlots = TremorGate::kNumStages + 1;   // stage별 + 합계

struct ProfileStat {
    uint32_t min, max;
    uint64_t sum;
};
static ProfileStat prof[kProfSlots];
static uint32_t prof_left = 0;

static uint32_t dwtCycles() { return DWT->CYCCNT; }

static void profileStart() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (ProfileStat& s : prof) s = ProfileStat{UINT32_MAX, 0, 0};
    prof_left = kProfileSamples;
    gate.setCycleCounter(dwtCycles);
}

static void profileAccumulate() {
    const uint32_t* c = gate.stageCycles();
    uint32_t total = 0;
    for (int i = 0; i < kProfSlots; ++i) {
        uint32_t v = (i < TremorGate::kNumStages) ? c[i] : total;
        if (i < TremorGate::kNumStages) total += v;
        ProfileStat& s = prof[i];
        if (v < s.min) s.min = v;
        if (v > s.max) s.max = v;
        s.sum += v;
    }
    if (--prof_left > 0) return;

    gate.setCycleCounter(nullptr);
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
    Serial.println(" % of sample period (pass <= 5%)");
}

// ---------------------------------------------------------------------------
// 출력
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

static void printCsv(uint32_t t_us, const ImuSample& s, const GateOutput& g, uint8_t bits) {
    char buf[160];
    char* p = buf;
    p = appendU32(p, seq);            *p++ = ',';
    p = appendU32(p, t_us);           *p++ = ',';
    p = appendFloat(p, s.ax, 4);      *p++ = ',';
    p = appendFloat(p, s.ay, 4);      *p++ = ',';
    p = appendFloat(p, s.az, 4);      *p++ = ',';
    p = appendFloat(p, s.gx, 3);      *p++ = ',';
    p = appendFloat(p, s.gy, 3);      *p++ = ',';
    p = appendFloat(p, s.gz, 3);      *p++ = ',';
    p = appendU32(p, bits);           *p++ = ',';
    p = appendU32(p, (uint8_t)hctrl.mode()); *p++ = ',';
    p = appendFloat(p, g.A, 3);       *p++ = ',';
    p = appendFloat(p, g.C, 3);       *p++ = ',';
    p = appendFloat(p, g.r, 3);       *p++ = ',';
    p = appendFloat(p, g.D, 3);       *p++ = ',';
    p = appendFloat(p, g.f_acf, 2);   *p++ = ',';
    *p++ = s.clipped ? '1' : '0';
    *p++ = '\r';
    *p++ = '\n';
    Serial.write((const uint8_t*)buf, p - buf);
}

static void printParams() {
    const GateParams& p = gate.params();
    Serial.print("# cfg_ver=");
    Serial.print(cfg_ver);
    Serial.print(" mode=");
    Serial.print((uint8_t)hctrl.mode());
    Serial.print(" trial=");
    Serial.print(hctrl.trialActive() ? 1 : 0);
    Serial.print(" cfg_valid=");
    Serial.print(gate.cfgValid() ? 1 : 0);
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

static void reply(bool ok, const char* line) {
    Serial.print(ok ? "# ok " : "# err ");
    Serial.println(line);
}

// ---------------------------------------------------------------------------
// Serial 명령 파서
// ---------------------------------------------------------------------------
static void handleLine(char* line) {
    switch (line[0]) {
        case 'm': {
            char* end;
            long v = strtol(line + 1, &end, 10);
            reply(end != line + 1 && *end == '\0' && v >= 0 && v <= 255 && cmd_set_mode((uint8_t)v), line);
            break;
        }
        case 'p': {
            char* eq = strchr(line + 1, '=');
            bool ok = false;
            if (eq && eq != line + 1) {
                *eq = '\0';
                char* end;
                float v = strtof(eq + 1, &end);
                ok = end != eq + 1 && *end == '\0' && cmd_set_param(line + 1, v);
                *eq = '=';
            }
            reply(ok, line);
            if (ok) printParams();
            break;
        }
        case 'r':
            cmd_reset_gate();
            reply(true, line);
            break;
        case 't': {
            bool ok = (line[1] == '0' || line[1] == '1') && line[2] == '\0';
            if (ok) cmd_set_trial(line[1] == '1');
            reply(ok, line);
            break;
        }
        case '?':
            printParams();
            break;
        case 'c':
            profileStart();
            reply(true, line);
            break;
        default:
            reply(false, line);
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
// 전원 표시 LED (XIAO nRF52840 RGB LED는 active-LOW)
//   USB 연결(VBUS 감지)  → 초록 ON
//   배터리만 연결        → 파랑 ON
static void powerLedInit() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    // 부팅 표시: 빨강 3회 깜빡임
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

void setup() {
    powerLedInit();
    // USB 없이 배터리로 켜도 멈추지 않도록 Serial 연결을 기다리지 않는다
    Serial.begin(kSerialBaud);

    if (!initIMU()) {
        Serial.println("# IMU init failed");
        while (1) delay(10);
    }
    Haptic_Init();

    Serial.println("# Tremor watch FW (spec v1.1)");
    printParams();
}

void loop() {
    powerLedUpdate();
    pollSerial();

    if (micros() - last_sample_us < kPollGuardUs) return;
    if (!imuDataReady()) return;
    const uint32_t t_us = micros();
    const ImuSample s = readIMU();
    last_sample_us = t_us;

    const GateOutput& g = gate.update(Vec3{s.gx, s.gy, s.gz});
    if (prof_left > 0) profileAccumulate();

    Haptic_Drive(hctrl.decide(g));
    uint8_t bits = g.bits;
    if (Haptic_IsActive()) bits |= GateBit::VIB_ON;

    printCsv(t_us, s, g, bits);
    ++seq;
}
