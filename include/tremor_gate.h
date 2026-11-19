#ifndef TREMOR_GATE_H
#define TREMOR_GATE_H

// Tremor gate core (FIRMWARE_SPEC §4). Platform-independent: no Arduino/Zephyr dependency.
// float32 only, no dynamic allocation.

#include <stdint.h>

struct Vec3 { float x, y, z; };   // gyro [deg/s]

// Status bits (§4.8). Written to BLE packets as-is.
namespace GateBit {
constexpr uint8_t S1        = 1u << 0;
constexpr uint8_t S2        = 1u << 1;
constexpr uint8_t S3        = 1u << 2;
constexpr uint8_t RAW       = 1u << 3;
constexpr uint8_t GATE_ON   = 1u << 4;
constexpr uint8_t VIB_ON    = 1u << 5;   // filled by main from the driver state, not by the gate
constexpr uint8_t CFG_VALID = 1u << 6;
constexpr uint8_t WARMUP    = 1u << 7;
}

// --- Biquad (Direct Form II Transposed, RBJ cookbook) ---
class Biquad {
  public:
    void setLowpass(float fc, float fs, float q);
    void setHighpass(float fc, float fs, float q);
    void setBandpass(float f0, float fs, float q);   // peak gain 0 dB

    float process(float in) {
        float out = b0_ * in + z1_;
        z1_ = b1_ * in - a1_ * out + z2_;
        z2_ = b2_ * in - a2_ * out;
        return out;
    }

    // Set the internal state to the steady state for a constant input `in` and return that output
    float prime(float in);
    void clear() { z1_ = z2_ = 0.0f; }

  private:
    void set(float b0, float b1, float b2, float a0, float a1, float a2);
    float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
};

// 4th-order Butterworth bandpass, one axis: 2 HPF stages + 2 LPF stages (Q 0.5412 / 1.3066)
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

// --- Parameters (§7) ---
struct GateParams {
    float fs             = 104.0f;
    float bp_lo          = 4.0f;
    float bp_hi          = 12.0f;
    float f0             = 0.0f;    // 0 = not set (calibration)
    float theta_on       = 1.0f;    // PLACEHOLDER, overwritten by calibration
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
    bool  theta_set      = false;   // whether theta_on was explicitly set (cfg_valid condition)
};

// Field table shared by Serial `p<name>=<value>` and BLE writes.
// fs, bp_lo and bp_hi are tied to the IMU configuration, so they are not writable.
struct ParamField {
    const char* name;
    float GateParams::*ptr;
};
extern const ParamField kParamFields[];
extern const int kNumParamFields;

bool GateParams_validate(const GateParams& p);
// Change one field by name. Modifies p and returns true only if the result passes validation.
bool GateParams_set(GateParams& p, const char* name, float value);

struct GateOutput {
    uint8_t bits  = 0;
    float   A     = 0.0f;   // Stage 2 RMS [deg/s]
    float   C     = 0.0f;   // Stage 1 concentration
    float   r     = 0.0f;   // Stage 3 ACF peak
    float   D     = 0.0f;   // peak-to-trough depth
    float   f_acf = 0.0f;   // ACF-based frequency [Hz]
};

class TremorGate {
  public:
    static constexpr int kMaxWin = 208;          // max rms/acf window [samples]
    static constexpr int kRing   = kMaxWin + 1;  // keeps one extra sample that leaves the window
    static constexpr int kMaxLag = 64;

    // Per-stage cycle profiling (§8). 0 preprocessing, 1 Stage1, 2 Stage2, 3 Stage3, 4 decision
    static constexpr int kNumStages = 5;
    typedef uint32_t (*CycleFn)();

    TremorGate();

    // Returns false and keeps the existing parameters if validation fails. On success, calls reset().
    bool setParams(const GateParams& p);
    const GateParams& params() const { return p_; }
    bool cfgValid() const { return p_.f0 > 0.0f && p_.theta_set; }

    // Resets filter state and buffers only (parameters kept).
    void reset();

    const GateOutput& update(const Vec3& gyro);   // once per sample
    const GateOutput& output() const { return out_; }

    // --- For tests / profiling ---
    int acfWindow() const { return N_; }
    int lagLo() const { return lag_lo_; }
    int lagHi() const { return lag_hi_; }
    int warmupSamples() const { return warmup_n_; }
    float acfR(int k) const { return r_[k]; }      // incrementally computed r(k)
    Vec3 windowSample(int i) const;                // i = 0: oldest sample in the ACF window
    void setCycleCounter(CycleFn fn) { cycle_fn_ = fn; }
    const uint32_t* stageCycles() const { return cyc_; }

  private:
    void configure();
    int idx(int back) const {   // ring index `back` samples ago
        int i = pos_ - back;
        return i < 0 ? i + kRing : i;
    }
    float dotBack(int a, int b) const {
        int i = idx(a), j = idx(b);
        return hx_[i] * hx_[j] + hy_[i] * hy_[j] + hz_[i] * hz_[j];
    }
    float lagExact(int k) const;   // window dot-product sum for lag k, computed from scratch
    void refreshLags(int n);       // overwrite n lags with exact values, round-robin
    void stage3();

    GateParams p_;
    GateOutput out_;

    // Derived values
    int   W_ = 0, N_ = 0;
    int   kmin_ = 0, kmax_ = 0, lag_lo_ = 0, lag_hi_ = 0;
    int   attack_n_ = 1, release_n_ = 1, warmup_n_ = 0;
    float alpha_ = 0.0f;

    // Preprocessing
    Bandpass4 bp_[3];
    Biquad    nb_[3];     // Stage 1 narrowband (f0, Q = f0/2)
    bool      primed_ = false;

    // Filter output history (3 axes + summed-axis energy).
    // Each value is written twice, at [pos_] and [pos_ + kRing], so the sample `b` back is always
    // at contiguous index pos_ + kRing − b (removes wrap from the dot-product loop).
    float hx_[2 * kRing], hy_[2 * kRing], hz_[2 * kRing], he_[2 * kRing];
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
    float acf_churn_ = 0.0f;   // energy added/removed since the current round-robin pass started
    int   acf_rr_ = 0;         // next lag to re-sum

    // Decision
    bool s1_ = false, s2_ = false, s3r_ = false, s3d_ = false;
    bool gate_ = false;
    int  run_ = 0;          // consecutive samples where RAW disagrees with the current state
    int  warm_cnt_ = 0;

    CycleFn  cycle_fn_ = nullptr;
    uint32_t cyc_[kNumStages] = {};
};

#endif
