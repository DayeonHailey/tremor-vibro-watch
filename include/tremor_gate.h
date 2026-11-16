#ifndef TREMOR_GATE_H
#define TREMOR_GATE_H

// Tremor 게이트 코어 (FIRMWARE_SPEC §4). 플랫폼 독립: Arduino·Zephyr 의존 없음.
// float32 전용, 동적 할당 없음.

#include <stdint.h>

struct Vec3 { float x, y, z; };   // 자이로 [deg/s]

// 상태 비트 (§4.8). BLE 패킷에 그대로 쓴다.
namespace GateBit {
constexpr uint8_t S1        = 1u << 0;
constexpr uint8_t S2        = 1u << 1;
constexpr uint8_t S3        = 1u << 2;
constexpr uint8_t RAW       = 1u << 3;
constexpr uint8_t GATE_ON   = 1u << 4;
constexpr uint8_t VIB_ON    = 1u << 5;   // 게이트가 아니라 main이 드라이버 상태로 채운다
constexpr uint8_t CFG_VALID = 1u << 6;
constexpr uint8_t WARMUP    = 1u << 7;
}

// --- Biquad (Direct Form II Transposed, RBJ cookbook) ---
class Biquad {
  public:
    void setLowpass(float fc, float fs, float q);
    void setHighpass(float fc, float fs, float q);
    void setBandpass(float f0, float fs, float q);   // 피크 이득 0 dB

    float process(float in) {
        float out = b0_ * in + z1_;
        z1_ = b1_ * in - a1_ * out + z2_;
        z2_ = b2_ * in - a2_ * out;
        return out;
    }

    // 입력 in이 무한히 지속된 정상 상태로 내부 상태를 맞추고 그 출력을 반환
    float prime(float in);
    void clear() { z1_ = z2_ = 0.0f; }

  private:
    void set(float b0, float b1, float b2, float a0, float a1, float a2);
    float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
};

// 4차 Butterworth 대역통과 한 축: HPF 2단 + LPF 2단 (Q 0.5412 / 1.3066)
class Bandpass4 {
  public:
    void design(float lo, float hi, float fs);
    float process(float in) {
        for (Biquad& s : s_) in = s.process(in);
        return in;
    }
    void prime(float in) {
        for (Biquad& s : s_) in = s.prime(in);
    }
    void clear() {
        for (Biquad& s : s_) s.clear();
    }

  private:
    Biquad s_[4];
};

// --- 파라미터 (§7) ---
struct GateParams {
    float fs             = 104.0f;
    float bp_lo          = 4.0f;
    float bp_hi          = 12.0f;
    float f0             = 0.0f;    // 0 = 미설정 (캘리브레이션)
    float theta_on       = 1.0f;    // PLACEHOLDER, 캘리브레이션이 덮어씀
    float theta_off_frac = 0.7f;
    float C_on           = 0.55f;   // PLACEHOLDER
    float C_off          = 0.40f;   // PLACEHOLDER
    float r_on           = 0.60f;   // PLACEHOLDER
    float r_off          = 0.45f;   // PLACEHOLDER
    float D_on           = 1.10f;   // PLACEHOLDER
    float D_off          = 0.80f;   // PLACEHOLDER
    float rms_win_s      = 0.5f;
    float acf_win_s      = 0.6f;
    float power_tau_s    = 0.3f;
    float attack_s       = 0.1f;
    float release_s      = 0.5f;
    bool  theta_set      = false;   // theta_on을 명시적으로 덮어썼는지 (cfg_valid 조건)
};

// Serial `p<name>=<value>`·BLE write가 공유하는 필드 테이블.
// fs·bp_lo·bp_hi는 IMU 설정과 묶여 있어서 쓰기 대상에서 뺀다.
struct ParamField {
    const char* name;
    float GateParams::*ptr;
};
extern const ParamField kParamFields[];
extern const int kNumParamFields;

bool GateParams_validate(const GateParams& p);
// 이름으로 필드 하나를 바꾼다. 결과가 검사를 통과할 때만 p를 고치고 true.
bool GateParams_set(GateParams& p, const char* name, float value);

struct GateOutput {
    uint8_t bits  = 0;
    float   A     = 0.0f;   // Stage 2 RMS [deg/s]
    float   C     = 0.0f;   // Stage 1 집중도
    float   r     = 0.0f;   // Stage 3 ACF 피크
    float   D     = 0.0f;   // 피크-골 깊이
    float   f_acf = 0.0f;   // ACF 기반 주파수 [Hz]
};

class TremorGate {
  public:
    static constexpr int kMaxWin = 208;          // rms·acf 창 상한 [샘플]
    static constexpr int kRing   = kMaxWin + 1;  // 창을 벗어나는 샘플 하나까지 보관
    static constexpr int kMaxLag = 64;

    // stage별 사이클 계측 (§8). 0 전처리, 1 Stage1, 2 Stage2, 3 Stage3, 4 판정
    static constexpr int kNumStages = 5;
    typedef uint32_t (*CycleFn)();

    TremorGate();

    // 검사에 실패하면 false를 반환하고 기존 파라미터를 유지한다. 성공하면 reset().
    bool setParams(const GateParams& p);
    const GateParams& params() const { return p_; }
    bool cfgValid() const { return p_.f0 > 0.0f && p_.theta_set; }

    // 필터 상태와 버퍼만 초기화한다 (파라미터 유지).
    void reset();

    const GateOutput& update(const Vec3& gyro);   // 샘플당 1회
    const GateOutput& output() const { return out_; }

    // --- 테스트·계측용 ---
    int acfWindow() const { return N_; }
    int lagLo() const { return lag_lo_; }
    int lagHi() const { return lag_hi_; }
    int warmupSamples() const { return warmup_n_; }
    float acfR(int k) const { return r_[k]; }      // 증분 계산 결과 r(k)
    Vec3 windowSample(int i) const;                // i = 0: ACF 창의 가장 오래된 샘플
    void setCycleCounter(CycleFn fn) { cycle_fn_ = fn; }
    const uint32_t* stageCycles() const { return cyc_; }

  private:
    void configure();
    int idx(int back) const {   // back 샘플 전의 링 인덱스
        int i = pos_ - back;
        return i < 0 ? i + kRing : i;
    }
    float dotBack(int a, int b) const {
        int i = idx(a), j = idx(b);
        return hx_[i] * hx_[j] + hy_[i] * hy_[j] + hz_[i] * hz_[j];
    }
    void recomputeAcf();
    void stage3();

    GateParams p_;
    GateOutput out_;

    // 파생값
    int   W_ = 0, N_ = 0;
    int   kmin_ = 0, kmax_ = 0, lag_lo_ = 0, lag_hi_ = 0;
    int   attack_n_ = 1, release_n_ = 1, warmup_n_ = 0;
    float alpha_ = 0.0f;

    // 전처리
    Bandpass4 bp_[3];
    Biquad    nb_[3];     // Stage 1 협대역 (f0, Q = f0/2)
    bool      primed_ = false;

    // 필터 출력 이력 (3축 + 축합 에너지)
    float hx_[kRing], hy_[kRing], hz_[kRing], he_[kRing];
    int   pos_ = 0;

    // Stage 1
    float pb_ = 0.0f, pn_ = 0.0f;
    // Stage 2
    float rms_sum_ = 0.0f;
    int   rms_lap_ = 0;
    // Stage 3
    float lag_sum_[kMaxLag + 1];
    float r_[kMaxLag + 1];
    float e1_[kMaxLag + 1], e2_[kMaxLag + 1];
    float acf_churn_ = 0.0f;   // 마지막 재합산 이후 들고 난 에너지
    int   acf_lap_ = 0;

    // 판정
    bool s1_ = false, s2_ = false, s3r_ = false, s3d_ = false;
    bool gate_ = false;
    int  run_ = 0;          // RAW가 현재 상태와 반대인 연속 샘플 수
    int  warm_cnt_ = 0;

    CycleFn  cycle_fn_ = nullptr;
    uint32_t cyc_[kNumStages] = {};
};

#endif
