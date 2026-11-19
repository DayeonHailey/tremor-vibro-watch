#include "tremor_gate.h"
#include <math.h>
#include <string.h>

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kQ1 = 0.5412f;   // 4th-order Butterworth split into 2 stages
constexpr float kQ2 = 1.3066f;

// Lower frequency bound for the Stage 3 lag search. With the spec's ceil(fs/4), a 4 Hz tremor (T = 26) sits on
// the search boundary; if noise pushes the peak to 27, detection fails, and the §11 f0 range (3.5–12 Hz) is not covered.
constexpr float kAcfSearchLoHz = 3.5f;
// Period-multiple correction: the first (shortest-lag) local maximum at or above this fraction of the highest one is the peak.
// Above 8 Hz both T and 2T fall in range; if r(2T) ≥ r(T), D collapses toward 0.
constexpr float kPeakRatio = 0.9f;
// If energy added/removed in the incremental sums exceeds this multiple of the window energy, re-sum before the lap ends.
// Prevents float cancellation error from growing relative to a small signal right after a large one leaves the window.
constexpr float kChurnLimit = 16.0f;
// Lags re-summed per sample once the churn limit is exceeded (29 lags → one pass in ~8 samples)
constexpr int kBurstRefresh = 4;

bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }   // rejects NaN

bool schmitt(bool state, float v, float on, float off) {
    return state ? (v >= off) : (v >= on);
}

int imax(int a, int b) { return a > b ? a : b; }

}  // namespace

// ---------------------------------------------------------------------------
// Biquad
// ---------------------------------------------------------------------------
void Biquad::set(float b0, float b1, float b2, float a0, float a1, float a2) {
    b0_ = b0 / a0;
    b1_ = b1 / a0;
    b2_ = b2 / a0;
    a1_ = a1 / a0;
    a2_ = a2 / a0;
}

void Biquad::setLowpass(float fc, float fs, float q) {
    float w0 = 2.0f * kPi * fc / fs;
    float cs = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    set((1.0f - cs) * 0.5f, 1.0f - cs, (1.0f - cs) * 0.5f, 1.0f + alpha, -2.0f * cs, 1.0f - alpha);
}

void Biquad::setHighpass(float fc, float fs, float q) {
    float w0 = 2.0f * kPi * fc / fs;
    float cs = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    set((1.0f + cs) * 0.5f, -(1.0f + cs), (1.0f + cs) * 0.5f, 1.0f + alpha, -2.0f * cs, 1.0f - alpha);
}

void Biquad::setBandpass(float f0, float fs, float q) {
    float w0 = 2.0f * kPi * f0 / fs;
    float cs = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    set(alpha, 0.0f, -alpha, 1.0f + alpha, -2.0f * cs, 1.0f - alpha);
}

float Biquad::prime(float in) {
    // DC steady state: y = H(1)·in, then solve for the DF2T state that produces that y
    float y = (b0_ + b1_ + b2_) / (1.0f + a1_ + a2_) * in;
    z1_ = y - b0_ * in;
    z2_ = b2_ * in - a2_ * y;
    return y;
}

void Bandpass4::design(float lo, float hi, float fs) {
    s_[0].setHighpass(lo, fs, kQ1);
    s_[1].setHighpass(lo, fs, kQ2);
    s_[2].setLowpass(hi, fs, kQ1);
    s_[3].setLowpass(hi, fs, kQ2);
}

// ---------------------------------------------------------------------------
// GateParams
// ---------------------------------------------------------------------------
const ParamField kParamFields[] = {
    {"f0", &GateParams::f0},
    {"theta_on", &GateParams::theta_on},
    {"theta_off_frac", &GateParams::theta_off_frac},
    {"C_on", &GateParams::C_on},
    {"C_off", &GateParams::C_off},
    {"r_on", &GateParams::r_on},
    {"r_off", &GateParams::r_off},
    {"D_on", &GateParams::D_on},
    {"D_off", &GateParams::D_off},
    {"rms_win_s", &GateParams::rms_win_s},
    {"acf_win_s", &GateParams::acf_win_s},
    {"power_tau_s", &GateParams::power_tau_s},
    {"attack_s", &GateParams::attack_s},
    {"release_s", &GateParams::release_s},
};
const int kNumParamFields = sizeof(kParamFields) / sizeof(kParamFields[0]);

bool GateParams_validate(const GateParams& p) {
    if (!inRange(p.fs, 26.0f, 416.0f)) return false;
    if (!inRange(p.bp_lo, 1.0f, p.fs * 0.45f)) return false;
    if (!inRange(p.bp_hi, p.bp_lo + 1.0f, p.fs * 0.45f)) return false;
    if (p.f0 != 0.0f && !inRange(p.f0, 3.5f, 12.0f)) return false;
    if (!inRange(p.theta_on, 1e-3f, 1000.0f)) return false;
    if (!inRange(p.theta_off_frac, 0.05f, 1.0f)) return false;
    if (!inRange(p.C_on, 0.0f, 1.0f) || !inRange(p.C_off, 0.0f, p.C_on)) return false;
    if (!inRange(p.r_on, 0.0f, 1.0f) || !inRange(p.r_off, 0.0f, p.r_on)) return false;
    if (!inRange(p.D_on, 0.0f, 2.0f) || !inRange(p.D_off, 0.0f, p.D_on)) return false;
    if (!inRange(p.power_tau_s, 0.02f, 5.0f)) return false;
    if (!inRange(p.attack_s, 0.0f, 2.0f) || !inRange(p.release_s, 0.0f, 10.0f)) return false;
    if (!inRange(p.rms_win_s, 0.05f, 2.0f) || !inRange(p.acf_win_s, 0.05f, 2.0f)) return false;

    long W = lroundf(p.rms_win_s * p.fs);
    long N = lroundf(p.acf_win_s * p.fs);
    int kmin = (int)floorf(p.fs / p.bp_hi);
    int kmax = (int)ceilf(p.fs / kAcfSearchLoHz);
    if (W < 4 || W > TremorGate::kMaxWin) return false;
    if (N > TremorGate::kMaxWin) return false;
    if (kmin < 2 || kmax + 1 > TremorGate::kMaxLag) return false;
    if (N < kmax + 3) return false;   // windows must still overlap at the largest lag
    return true;
}

bool GateParams_set(GateParams& p, const char* name, float value) {
    for (int i = 0; i < kNumParamFields; ++i) {
        const ParamField& f = kParamFields[i];
        if (strcmp(name, f.name) != 0) continue;
        GateParams q = p;
        q.*(f.ptr) = value;
        if (f.ptr == &GateParams::theta_on) q.theta_set = true;
        if (!GateParams_validate(q)) return false;
        p = q;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// TremorGate
// ---------------------------------------------------------------------------
TremorGate::TremorGate() {
    configure();
    reset();
}

bool TremorGate::setParams(const GateParams& p) {
    if (!GateParams_validate(p)) return false;
    p_ = p;
    configure();
    reset();
    return true;
}

void TremorGate::configure() {
    const float fs = p_.fs;
    W_ = (int)lroundf(p_.rms_win_s * fs);
    N_ = (int)lroundf(p_.acf_win_s * fs);
    kmin_ = (int)floorf(fs / p_.bp_hi);
    kmax_ = (int)ceilf(fs / kAcfSearchLoHz);
    // Widen the lag range as needed for r(T/2) and boundary-neighbor comparisons (§4.6)
    lag_lo_ = imax(1, kmin_ / 2 - 1);
    lag_hi_ = kmax_ + 1;

    attack_n_  = imax(1, (int)lroundf(p_.attack_s * fs));
    release_n_ = imax(1, (int)lroundf(p_.release_s * fs));
    alpha_ = 1.0f - expf(-1.0f / (p_.power_tau_s * fs));

    // Filter settling: the longer of 2 periods of the bandpass low cutoff and 3τ of the power EMA
    float settle_s = 2.0f / p_.bp_lo;
    if (3.0f * p_.power_tau_s > settle_s) settle_s = 3.0f * p_.power_tau_s;
    warmup_n_ = imax(imax(W_, N_), (int)ceilf(settle_s * fs));

    for (int a = 0; a < 3; ++a) {
        bp_[a].design(p_.bp_lo, p_.bp_hi, fs);
        if (p_.f0 > 0.0f) nb_[a].setBandpass(p_.f0, fs, p_.f0 * 0.5f);
    }
}

void TremorGate::reset() {
    for (int a = 0; a < 3; ++a) {
        bp_[a].clear();
        nb_[a].clear();
    }
    primed_ = false;
    memset(hx_, 0, sizeof(hx_));
    memset(hy_, 0, sizeof(hy_));
    memset(hz_, 0, sizeof(hz_));
    memset(he_, 0, sizeof(he_));
    memset(lag_sum_, 0, sizeof(lag_sum_));
    memset(r_, 0, sizeof(r_));
    memset(e1_, 0, sizeof(e1_));
    memset(e2_, 0, sizeof(e2_));
    pos_ = 0;
    pb_ = pn_ = 0.0f;
    rms_sum_ = 0.0f;
    rms_lap_ = 0;
    acf_churn_ = 0.0f;
    acf_rr_ = lag_lo_;
    s1_ = s2_ = s3r_ = s3d_ = false;
    gate_ = false;
    run_ = 0;
    warm_cnt_ = 0;
    out_ = GateOutput();
}

Vec3 TremorGate::windowSample(int i) const {
    int j = idx(N_ - 1 - i);
    return Vec3{hx_[j], hy_[j], hz_[j]};
}

const GateOutput& TremorGate::update(const Vec3& g) {
    uint32_t t = cycle_fn_ ? cycle_fn_() : 0;
    auto mark = [&](int stage) {
        if (!cycle_fn_) return;
        uint32_t now = cycle_fn_();
        cyc_[stage] = now - t;
        t = now;
    };

    // --- Common preprocessing: per-axis 4–12 Hz bandpass (§4.2) ---
    if (!primed_) {
        bp_[0].prime(g.x);
        bp_[1].prime(g.y);
        bp_[2].prime(g.z);
        primed_ = true;
    }
    const float x = bp_[0].process(g.x);
    const float y = bp_[1].process(g.y);
    const float z = bp_[2].process(g.z);
    const float e = x * x + y * y + z * z;
    pos_ = (pos_ + 1 == kRing) ? 0 : pos_ + 1;
    hx_[pos_] = hx_[pos_ + kRing] = x;
    hy_[pos_] = hy_[pos_ + kRing] = y;
    hz_[pos_] = hz_[pos_ + kRing] = z;
    he_[pos_] = he_[pos_ + kRing] = e;
    mark(0);

    // --- Stage 1: spectral concentration (§4.4) ---
    pb_ += alpha_ * (e - pb_);
    if (p_.f0 > 0.0f) {
        float u = nb_[0].process(x), v = nb_[1].process(y), w = nb_[2].process(z);
        pn_ += alpha_ * (u * u + v * v + w * w - pn_);
        float c = pb_ > 1e-12f ? pn_ / pb_ : 0.0f;
        out_.C = c > 1.0f ? 1.0f : c;
    } else {
        out_.C = 0.0f;
    }
    mark(1);

    // --- Stage 2: windowed RMS (§4.5) ---
    rms_sum_ += e - he_[idx(W_)];
    if (++rms_lap_ >= W_) {   // re-sum once per lap to remove accumulated error
        rms_lap_ = 0;
        float s = 0.0f;
        for (int j = 0; j < W_; ++j) s += he_[idx(j)];
        rms_sum_ = s;
    }
    out_.A = sqrtf((rms_sum_ > 0.0f ? rms_sum_ : 0.0f) / (float)W_);
    mark(2);

    // --- Stage 3: vector-normalized ACF (§4.6) ---
    stage3();
    mark(3);

    // --- Decision (§4.3, §4.7) ---
    const GateParams& p = p_;
    s1_  = schmitt(s1_, out_.C, p.C_on, p.C_off);
    s2_  = schmitt(s2_, out_.A, p.theta_on, p.theta_on * p.theta_off_frac);
    s3r_ = schmitt(s3r_, out_.r, p.r_on, p.r_off);
    s3d_ = schmitt(s3d_, out_.D, p.D_on, p.D_off);
    const bool s3  = s3r_ && s3d_;
    const bool cfg = cfgValid();
    const bool raw = s1_ && s2_ && s3 && cfg;

    const bool warm = warm_cnt_ < warmup_n_;
    if (warm) {
        ++warm_cnt_;
        gate_ = false;
        run_ = 0;
    } else if (raw != gate_) {
        if (++run_ >= (gate_ ? release_n_ : attack_n_)) {
            gate_ = raw;
            run_ = 0;
        }
    } else {
        run_ = 0;
    }

    uint8_t bits = 0;
    if (s1_)   bits |= GateBit::S1;
    if (s2_)   bits |= GateBit::S2;
    if (s3)    bits |= GateBit::S3;
    if (raw)   bits |= GateBit::RAW;
    if (gate_) bits |= GateBit::GATE_ON;
    if (cfg)   bits |= GateBit::CFG_VALID;
    if (warm)  bits |= GateBit::WARMUP;
    out_.bits = bits;
    mark(4);
    return out_;
}

float TremorGate::lagExact(int k) const {
    // Sum products x[n−j]·x[n−j−k], j = 0..N−k−1 within the window as a contiguous range of the mirror buffer.
    // The sample b back is at index pos_ + kRing − b.
    const int len = N_ - k;
    const int hi = pos_ + kRing;   // back 0
    const int lo = hi - (len - 1);  // back len−1
    const float* ax = &hx_[lo];
    const float* ay = &hy_[lo];
    const float* az = &hz_[lo];
    const float* bx = ax - k;
    const float* by = ay - k;
    const float* bz = az - k;
    // Four accumulators break the dependency chain (uses the FPU pipeline)
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    int i = 0;
    for (; i + 1 < len; i += 2) {
        s0 += ax[i] * bx[i] + ay[i] * by[i];
        s1 += az[i] * bz[i];
        s2 += ax[i + 1] * bx[i + 1] + ay[i + 1] * by[i + 1];
        s3 += az[i + 1] * bz[i + 1];
    }
    if (i < len) s0 += ax[i] * bx[i] + ay[i] * by[i] + az[i] * bz[i];
    return (s0 + s1) + (s2 + s3);
}

void TremorGate::refreshLags(int n) {
    // Re-summing all lags at once costs tens of thousands of cycles on that one sample (the cause of the T10 max).
    // Instead, overwrite a few lags per sample with exact values, round-robin, to cut accumulated error.
    // Each lag is refreshed every (lag_hi − lag_lo + 1)/n samples.
    for (int i = 0; i < n; ++i) {
        lag_sum_[acf_rr_] = lagExact(acf_rr_);
        if (++acf_rr_ > lag_hi_) {
            acf_rr_ = lag_lo_;
            acf_churn_ = 0.0f;   // after a full pass every lag holds a fresh value
        }
    }
}

void TremorGate::stage3() {
    // Incremental update: add the new product x[n]·x[n-k], subtract x[n-N+k]·x[n-N] that left the window
    for (int k = lag_lo_; k <= lag_hi_; ++k) {
        lag_sum_[k] += dotBack(0, k) - dotBack(N_ - k, N_);
    }
    acf_churn_ += he_[pos_] + he_[idx(N_)];

    // E1(k) = energy of the newest N−k samples, E2(k) = energy of the oldest N−k samples.
    // Accumulated directly at both ends rather than subtracted from the total (avoids cancellation when a large burst sits at one end).
    float tail = 0.0f, head = 0.0f;
    for (int m = 1; m <= N_; ++m) {
        tail += he_[idx(m - 1)];
        head += he_[idx(N_ - m)];
        const int k = N_ - m;
        if (k >= lag_lo_ && k <= lag_hi_) {
            e1_[k] = tail;
            e2_[k] = head;
        }
    }
    const float e_tot = tail;

    // Normally 1 per sample; cycle quickly when large energy has entered/left and cancellation error may grow
    refreshLags(acf_churn_ > kChurnLimit * e_tot ? kBurstRefresh : 1);

    for (int k = lag_lo_; k <= lag_hi_; ++k) {
        const float d = e1_[k] * e2_[k];
        const float r = d > 1e-20f ? lag_sum_[k] / sqrtf(d) : 0.0f;
        r_[k] = r > 1.0f ? 1.0f : (r < -1.0f ? -1.0f : r);
    }

    // Local maxima within [k_min, k_max]. Neighbors are compared in the widened range, so
    // monotonically decreasing/increasing boundary values are not peaks.
    float r_max = -2.0f;
    for (int k = kmin_; k <= kmax_; ++k) {
        if (r_[k] >= r_[k - 1] && r_[k] >= r_[k + 1] && r_[k] > r_max) r_max = r_[k];
    }
    if (r_max < -1.5f) {
        out_.r = out_.D = out_.f_acf = 0.0f;
        return;
    }
    const float r_need = r_max > 0.0f ? kPeakRatio * r_max : r_max;
    int best = kmin_;
    while (!(r_[best] >= r_[best - 1] && r_[best] >= r_[best + 1] && r_[best] >= r_need)) ++best;

    const float rm = r_[best - 1], r0 = r_[best], rp = r_[best + 1];
    const float den = rm - 2.0f * r0 + rp;
    float delta = den < -1e-9f ? 0.5f * (rm - rp) / den : 0.0f;
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;
    const float T = (float)best + delta;

    int half = (int)lroundf(0.5f * T);
    if (half < lag_lo_) half = lag_lo_;
    out_.r = r0;
    out_.D = r0 - r_[half];
    out_.f_acf = p_.fs / T;
}
