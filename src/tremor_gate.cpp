#include "tremor_gate.h"
#include <math.h>
#include <string.h>

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kQ1 = 0.5412f;   // 4차 Butterworth 2단 분해
constexpr float kQ2 = 1.3066f;

// Stage 3 lag 탐색 하한 주파수. 명세의 ceil(fs/4)이면 4 Hz tremor(T = 26)가 탐색 경계에
// 걸려 노이즈에 따라 피크가 27로 밀리면 검출에 실패하고, §11의 f0 범위(3.5–12 Hz)도 못 덮는다.
constexpr float kAcfSearchLoHz = 3.5f;
// 주기 배수 보정: 가장 높은 국소 최대의 이 비율 이상인 첫(최단 lag) 국소 최대를 피크로 한다.
// 8 Hz 이상이면 T와 2T가 둘 다 범위 안에 들어와서 r(2T) ≥ r(T)가 되면 D가 0 근처로 무너진다.
constexpr float kPeakRatio = 0.9f;
// 증분 합에 들고 난 에너지가 창 에너지의 이 배를 넘으면 lap 전이라도 재합산한다.
// 큰 신호가 빠져나간 직후 작은 신호 기준으로 float 상쇄 오차가 커지는 것을 막는다.
constexpr float kChurnLimit = 16.0f;

bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }   // NaN은 거부

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
    // DC 정상 상태: y = H(1)·in, 그 y를 내는 DF2T 상태를 역산
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
    if (N < kmax + 3) return false;   // 최대 lag에서도 겹치는 구간이 남아야 한다
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
    // r(T/2)와 경계의 이웃 비교에 필요한 만큼 lag 범위를 넓힌다 (§4.6)
    lag_lo_ = imax(1, kmin_ / 2 - 1);
    lag_hi_ = kmax_ + 1;

    attack_n_  = imax(1, (int)lroundf(p_.attack_s * fs));
    release_n_ = imax(1, (int)lroundf(p_.release_s * fs));
    alpha_ = 1.0f - expf(-1.0f / (p_.power_tau_s * fs));

    // 필터 안정화: 대역통과 하한 2주기, power EMA 3τ 중 긴 쪽
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
    acf_lap_ = 0;
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

    // --- 공통 전처리: 축별 4–12 Hz 대역통과 (§4.2) ---
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
    hx_[pos_] = x;
    hy_[pos_] = y;
    hz_[pos_] = z;
    he_[pos_] = e;
    mark(0);

    // --- Stage 1: 스펙트럼 집중도 (§4.4) ---
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

    // --- Stage 2: 창 RMS (§4.5) ---
    rms_sum_ += e - he_[idx(W_)];
    if (++rms_lap_ >= W_) {   // 한 바퀴마다 재합산해서 누적 오차 제거
        rms_lap_ = 0;
        float s = 0.0f;
        for (int j = 0; j < W_; ++j) s += he_[idx(j)];
        rms_sum_ = s;
    }
    out_.A = sqrtf((rms_sum_ > 0.0f ? rms_sum_ : 0.0f) / (float)W_);
    mark(2);

    // --- Stage 3: 벡터 정규화 ACF (§4.6) ---
    stage3();
    mark(3);

    // --- 판정 (§4.3, §4.7) ---
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

void TremorGate::recomputeAcf() {
    for (int k = lag_lo_; k <= lag_hi_; ++k) {
        float s = 0.0f;
        for (int j = 0; j + k < N_; ++j) s += dotBack(j, j + k);
        lag_sum_[k] = s;
    }
}

void TremorGate::stage3() {
    // 증분 갱신: 새 곱 x[n]·x[n-k]를 더하고, 창을 벗어난 x[n-N+k]·x[n-N]을 뺀다
    for (int k = lag_lo_; k <= lag_hi_; ++k) {
        lag_sum_[k] += dotBack(0, k) - dotBack(N_ - k, N_);
    }
    acf_churn_ += he_[pos_] + he_[idx(N_)];

    // E1(k) = 최신 N−k샘플, E2(k) = 가장 오래된 N−k샘플의 에너지.
    // 전체에서 빼지 않고 양 끝에서 직접 누적한다 (큰 버스트가 한쪽에 몰렸을 때 상쇄 오차 방지).
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

    if (++acf_lap_ >= N_ || acf_churn_ > kChurnLimit * e_tot) {
        acf_lap_ = 0;
        acf_churn_ = 0.0f;
        recomputeAcf();
    }

    for (int k = lag_lo_; k <= lag_hi_; ++k) {
        const float d = e1_[k] * e2_[k];
        const float r = d > 1e-20f ? lag_sum_[k] / sqrtf(d) : 0.0f;
        r_[k] = r > 1.0f ? 1.0f : (r < -1.0f ? -1.0f : r);
    }

    // [k_min, k_max] 안의 국소 최대. 이웃은 넓힌 범위에서 비교하므로
    // 단조 감소·증가하는 경계값은 피크가 되지 않는다.
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
