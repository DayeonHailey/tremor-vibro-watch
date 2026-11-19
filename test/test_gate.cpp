// Host tests (FIRMWARE_SPEC §9). No device needed.
//   cd test && make && ./test_gate
// or: g++ -std=c++17 -O2 -I../include test_gate.cpp ../src/tremor_gate.cpp -o test_gate
//
// The test harness uses double for reference values (firmware code is float-only).

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <random>
#include <vector>

#include "haptic_controller.h"
#include "tremor_gate.h"

namespace {

constexpr float kFs    = 104.0f;
constexpr float kTheta = 3.0f;   // θ_on for tests [deg/s]
constexpr double kPi   = 3.14159265358979323846;

int g_failed = 0;

void report(bool ok, const char* name, const char* fmt, ...) {
    std::printf("[%s] %-4s ", ok ? "PASS" : "FAIL", name);
    va_list ap;
    va_start(ap, fmt);
    std::vprintf(fmt, ap);
    va_end(ap);
    std::printf("\n");
    if (!ok) ++g_failed;
}

GateParams baseParams(float f0) {
    GateParams p;
    p.f0 = f0;
    p.theta_on = kTheta;
    p.theta_set = true;
    return p;
}

// Amplitude gain of the 4–12 Hz bandpass at frequency f (steady-state RMS ratio)
double bandGain(float f) {
    Bandpass4 bp;
    bp.design(4.0f, 12.0f, kFs);
    double pin = 0, pout = 0;
    const int total = (int)(20 * kFs), skip = (int)(10 * kFs);
    for (int n = 0; n < total; ++n) {
        float x = (float)std::sin(2 * kPi * f * n / kFs);
        float y = bp.process(x);
        if (n >= skip) {
            pin += x * x;
            pout += y * y;
        }
    }
    return std::sqrt(pout / pin);
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)std::ceil(q * v.size());
    return v[i == 0 ? 0 : i - 1];
}

// --- Synthetic 3-axis gyro signal ---------------------------------------------
struct Synth {
    std::mt19937 rng;
    std::normal_distribution<float> nd{0.0f, 1.0f};

    float noise_band_rms = 0.0f;   // target vector RMS in the 4–12 Hz band (independent white noise per axis)
    Vec3  bias{0, 0, 0};

    float tremor_rms = 0.0f;       // vector RMS of the tremor component (= contribution to band RMS A)
    float f_tremor   = 5.0f;
    float jitter     = 0.5f;       // phase random walk [rad/sqrt(s)]
    Vec3  tdir{1, 0, 0};

    float vol_amp  = 0.0f;         // voluntary movement amplitude [deg/s]
    float vol_f_lo = 1.0f, vol_f_hi = 2.5f;
    Vec3  vdir{0, 1, 0};

    double tphase = 0, vphase = 0, t = 0;

    explicit Synth(unsigned seed) : rng(seed) {}

    Vec3 unitRandom() {
        Vec3 v{nd(rng), nd(rng), nd(rng)};
        float n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return Vec3{v.x / n, v.y / n, v.z / n};
    }

    Vec3 next() {
        const double dt = 1.0 / kFs;
        // band power ratio = 3 axes × (12−4)/(fs/2)
        const float sw = noise_band_rms / std::sqrt(3.0f * 8.0f / (kFs * 0.5f));
        tphase += 2 * kPi * f_tremor * dt + jitter * nd(rng) * std::sqrt(dt);
        const double fv = 0.5 * (vol_f_lo + vol_f_hi) + 0.5 * (vol_f_hi - vol_f_lo) * std::sin(2 * kPi * 0.07 * t);
        vphase += 2 * kPi * fv * dt;
        t += dt;
        const float tr = (float)(std::sqrt(2.0) * tremor_rms * std::sin(tphase));
        const float vo = (float)(vol_amp * std::sin(vphase));
        return Vec3{bias.x + tr * tdir.x + vo * vdir.x + sw * nd(rng),
                    bias.y + tr * tdir.y + vo * vdir.y + sw * nd(rng),
                    bias.z + tr * tdir.z + vo * vdir.z + sw * nd(rng)};
    }
};

struct BitStats {
    long n = 0, s1 = 0, s2 = 0, s3 = 0, raw = 0, on = 0;
    void add(uint8_t b) {
        ++n;
        s1 += !!(b & GateBit::S1);
        s2 += !!(b & GateBit::S2);
        s3 += !!(b & GateBit::S3);
        raw += !!(b & GateBit::RAW);
        on += !!(b & GateBit::GATE_ON);
    }
    double frac(long v) const { return n ? (double)v / n : 0.0; }
};

// --- T0: basic behavior --------------------------------------------------------
void testSanity() {
    {
        TremorGate g;
        GateParams p = g.params();
        bool ok = !g.cfgValid();
        ok = ok && GateParams_set(p, "f0", 5.0f) && g.setParams(p) && !g.cfgValid();
        ok = ok && GateParams_set(p, "theta_on", kTheta) && g.setParams(p) && g.cfgValid();
        report(ok, "T0a", "cfg_valid turns on only when both f0 and theta_on are set");
    }
    {
        GateParams p = baseParams(5.0f);
        bool ok = !GateParams_set(p, "f0", 20.0f) && !GateParams_set(p, "C_off", 0.9f) &&
                  !GateParams_set(p, "nope", 1.0f) && !GateParams_set(p, "acf_win_s", 0.2f) &&
                  !GateParams_set(p, "theta_on", NAN) && p.f0 == 5.0f && p.C_off == 0.40f;
        ok = ok && GateParams_set(p, "C_on", 0.6f) && p.C_on == 0.6f;
        report(ok, "T0b", "out-of-range values and unknown fields are rejected, existing values kept");
    }
    {
        TremorGate g;
        g.setParams(baseParams(5.0f));
        Synth s(1);
        s.tremor_rms = 2 * kTheta;
        s.jitter = 0;
        s.f_tremor = 5.0f;
        bool warm_ok = true;
        for (int n = 0; n < g.warmupSamples(); ++n) {
            uint8_t b = g.update(s.next()).bits;
            warm_ok = warm_ok && (b & GateBit::WARMUP) && !(b & GateBit::GATE_ON);
        }
        for (int n = 0; n < 3 * kFs; ++n) g.update(s.next());
        const GateOutput& o = g.output();
        bool ok = warm_ok && o.C > 0.95f && o.r > 0.95f && o.D > 1.8f && std::fabs(o.f_acf - 5.0f) < 0.2f &&
                  (o.bits & GateBit::GATE_ON) && !(o.bits & GateBit::WARMUP);
        report(ok, "T0c", "pure 5 Hz: C=%.3f r=%.3f D=%.3f f_acf=%.2f, forced OFF for %d warmup samples=%s", o.C, o.r, o.D,
               o.f_acf, g.warmupSamples(), warm_ok ? "yes" : "no");

        g.reset();
        uint8_t b = g.update(s.next()).bits;
        report((b & GateBit::WARMUP) && g.params().f0 == 5.0f && g.cfgValid(), "T0d",
               "reset() keeps parameters and re-enters warmup");
    }
}

// --- T1: filter response --------------------------------------------------------
void testFilter() {
    const float freqs[] = {4.0f, 6.0f, 8.0f, 12.0f};
    const float expect[] = {-3.0f, NAN, 0.0f, -3.0f};
    bool ok = true;
    char msg[160];
    int len = 0;
    for (int i = 0; i < 4; ++i) {
        double db = 20 * std::log10(bandGain(freqs[i]));
        if (!std::isnan(expect[i])) ok = ok && std::fabs(db - expect[i]) <= 1.0;
        len += std::snprintf(msg + len, sizeof(msg) - len, "%g Hz %.2f dB  ", freqs[i], db);
    }
    report(ok, "T1", "%s(edges −3±1, center 0±1)", msg);
}

// --- T2: latency -----------------------------------------------------------------
// The tremor amplitude 1.5θ is the post-bandpass RMS. Since θ is set from the same filter output during
// calibration (A_post, §11), the raw signal is scaled up by the gain at the band edge (4 Hz, −3 dB).
void testLatency() {
    const float f0s[] = {4, 5, 6, 8, 10};
    const int seeds = 100;
    std::vector<double> all;
    for (int fi = 0; fi < 5; ++fi) {
        const float f0 = f0s[fi];
        const float amp = (float)(1.5 * kTheta / bandGain(f0));
        std::vector<double> lat, off, st_lat[3];
        int pre_on = 0;
        for (int seed = 0; seed < seeds; ++seed) {
            TremorGate g;
            g.setParams(baseParams(f0));
            Synth s(1000u * fi + seed);
            s.noise_band_rms = 0.25f * kTheta;
            s.bias = Vec3{s.nd(s.rng) * 3, s.nd(s.rng) * 3, s.nd(s.rng) * 3};
            s.tdir = s.unitRandom();
            s.f_tremor = f0;
            const int on = (int)(4 * kFs), stop = (int)(7 * kFs), end = (int)(10 * kFs);
            int first_on = -1, first_off = -1, first_st[3] = {-1, -1, -1};
            const uint8_t st_bit[3] = {GateBit::S1, GateBit::S2, GateBit::S3};
            bool pre = false;
            for (int n = 0; n < end; ++n) {
                s.tremor_rms = (n >= on && n < stop) ? amp : 0.0f;
                const uint8_t b = g.update(s.next()).bits;
                const bool gate = b & GateBit::GATE_ON;
                if (n < on && gate) pre = true;
                if (n >= on && first_on < 0 && gate) first_on = n;
                if (n >= stop && first_off < 0 && !gate) first_off = n;
                for (int i = 0; i < 3; ++i)
                    if (n >= on && first_st[i] < 0 && (b & st_bit[i])) first_st[i] = n;
            }
            pre_on += pre;
            for (int i = 0; i < 3; ++i) st_lat[i].push_back(first_st[i] < 0 ? INFINITY : (first_st[i] - on) / kFs);
            lat.push_back(first_on < 0 ? INFINITY : (first_on - on) / kFs);
            if (first_off >= 0) off.push_back((first_off - stop) / kFs);
            all.push_back(lat.back());
        }
        const double p95 = percentile(lat, 0.95);
        report(p95 <= 0.75 && pre_on == 0, "T2", "f0=%4.1f Hz onset p50=%.3f p95=%.3f max=%.3f s | offset p50=%.3f p95=%.3f s | early false ON %d/%d",
               f0, percentile(lat, 0.5), p95, percentile(lat, 1.0), percentile(off, 0.5), percentile(off, 0.95),
               pre_on, seeds);
        std::printf("            stage first-pass p95: S1 %.3f  S2 %.3f  S3 %.3f s\n", percentile(st_lat[0], 0.95),
                    percentile(st_lat[1], 0.95), percentile(st_lat[2], 0.95));
    }
    report(percentile(all, 0.95) <= 0.75, "T2", "overall onset p95=%.3f s (target ≤ 0.75)", percentile(all, 0.95));
}

// --- Common: run for a fixed time and collect bit statistics ---------------------
BitStats runFor(TremorGate& g, Synth& s, float seconds) {
    BitStats st;
    const int n = (int)(seconds * kFs);
    for (int i = 0; i < n; ++i) {
        uint8_t b = g.update(s.next()).bits;
        if (!(b & GateBit::WARMUP)) st.add(b);
    }
    return st;
}

// --- T3: noise only (below θ) -----------------------------------------------------
void testNoiseFalseAlarm() {
    TremorGate g;
    g.setParams(baseParams(5.0f));
    Synth s(31);
    s.noise_band_rms = 0.6f * kTheta;
    s.bias = Vec3{1.5f, -2.0f, 0.7f};
    BitStats st = runFor(g, s, 600.0f);
    report(st.frac(st.on) < 0.01, "T3", "noise 0.6θ 10 min: GATE_ON %.4f%% (S1 %.1f%% S2 %.1f%% S3 %.1f%%)",
           100 * st.frac(st.on), 100 * st.frac(st.s1), 100 * st.frac(st.s2), 100 * st.frac(st.s3));
}

// --- T4: voluntary movement (out of band, large amplitude) ------------------------
void testVoluntary() {
    const float f0s[] = {4.0f, 5.0f, 8.0f};
    for (float f0 : f0s) {
        TremorGate g;
        g.setParams(baseParams(f0));
        Synth s(41 + (unsigned)f0);
        s.noise_band_rms = 0.25f * kTheta;
        s.vol_amp = 100.0f;
        s.vol_f_lo = 1.0f;
        s.vol_f_hi = 2.5f;
        s.vdir = s.unitRandom();
        BitStats st = runFor(g, s, 120.0f);
        report(st.frac(st.on) < 0.01, "T4", "f0=%g, voluntary 100 deg/s 1–2.5 Hz 2 min: GATE_ON %.3f%% (S1 %.1f%% S2 %.1f%% S3 %.1f%%)",
               f0, 100 * st.frac(st.on), 100 * st.frac(st.s1), 100 * st.frac(st.s2), 100 * st.frac(st.s3));
    }
}

// --- T5: tremor + voluntary movement -----------------------------------------------
void testTremorPlusVoluntary() {
    const float f0s[] = {4.0f, 6.0f, 10.0f};
    for (float f0 : f0s) {
        TremorGate g;
        g.setParams(baseParams(f0));
        Synth s(51 + (unsigned)f0);
        s.noise_band_rms = 0.25f * kTheta;
        s.tremor_rms = 2.0f * kTheta;
        s.f_tremor = f0;
        s.tdir = s.unitRandom();
        s.vol_amp = 40.0f;
        s.vol_f_lo = 1.0f;
        s.vol_f_hi = 2.0f;
        s.vdir = s.unitRandom();
        runFor(g, s, 3.0f);
        BitStats st = runFor(g, s, 60.0f);
        bool ok = st.frac(st.on) >= 0.95 && st.frac(st.s1) >= 0.95 && st.frac(st.s2) >= 0.95 && st.frac(st.s3) >= 0.95;
        report(ok, "T5", "f0=%g, tremor 2θ + voluntary 40 deg/s 1–2 Hz: GATE_ON %.1f%% (S1 %.1f%% S2 %.1f%% S3 %.1f%%)",
               f0, 100 * st.frac(st.on), 100 * st.frac(st.s1), 100 * st.frac(st.s2), 100 * st.frac(st.s3));
    }
}

// --- T6: axis invariance -----------------------------------------------------------
struct Mat3 { float m[3][3]; };

Mat3 randomRotation(std::mt19937& rng) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    double u1 = u(rng), u2 = u(rng), u3 = u(rng);   // Shoemake uniform quaternion
    double a = std::sqrt(1 - u1), b = std::sqrt(u1);
    double w = a * std::sin(2 * kPi * u2), x = a * std::cos(2 * kPi * u2);
    double y = b * std::sin(2 * kPi * u3), z = b * std::cos(2 * kPi * u3);
    Mat3 r;
    r.m[0][0] = (float)(1 - 2 * (y * y + z * z)); r.m[0][1] = (float)(2 * (x * y - z * w)); r.m[0][2] = (float)(2 * (x * z + y * w));
    r.m[1][0] = (float)(2 * (x * y + z * w)); r.m[1][1] = (float)(1 - 2 * (x * x + z * z)); r.m[1][2] = (float)(2 * (y * z - x * w));
    r.m[2][0] = (float)(2 * (x * z - y * w)); r.m[2][1] = (float)(2 * (y * z + x * w)); r.m[2][2] = (float)(1 - 2 * (x * x + y * y));
    return r;
}

Vec3 mul(const Mat3& r, const Vec3& v) {
    return Vec3{r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
                r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
                r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z};
}

std::vector<uint8_t> runSequence(const std::vector<Vec3>& seq, const Mat3* rot) {
    TremorGate g;
    g.setParams(baseParams(5.0f));
    std::vector<uint8_t> bits;
    bits.reserve(seq.size());
    for (const Vec3& v : seq) bits.push_back(g.update(rot ? mul(*rot, v) : v).bits);
    return bits;
}

void testAxisInvariance() {
    // Tremor only along the sensor z axis (gravity-normal direction of v0), toggled on/off at varying amplitude
    Synth s(61);
    s.noise_band_rms = 0.3f * kTheta;
    s.bias = Vec3{2.0f, -1.0f, 0.5f};
    s.tdir = Vec3{0, 0, 1};
    s.f_tremor = 5.0f;
    const float amps[] = {1.2f, 0.0f, 2.0f, 0.0f, 0.8f, 0.0f, 3.0f, 0.0f, 1.5f, 0.0f, 1.0f, 0.0f};
    std::vector<Vec3> seq;
    for (int rep = 0; rep < 2; ++rep)
        for (float a : amps) {
            s.tremor_rms = a * kTheta;
            for (int n = 0; n < (int)(5 * kFs); ++n) seq.push_back(s.next());
        }

    std::vector<uint8_t> ref = runSequence(seq, nullptr);
    long ref_on = 0, valid = 0;
    for (uint8_t b : ref)
        if (!(b & GateBit::WARMUP)) {
            ++valid;
            ref_on += !!(b & GateBit::RAW);
        }

    std::mt19937 rng(62);
    double worst = 1.0;
    for (int t = 0; t < 20; ++t) {
        Mat3 r = randomRotation(rng);
        std::vector<uint8_t> b = runSequence(seq, &r);
        long match = 0;
        for (size_t i = 0; i < ref.size(); ++i)
            if (!(ref[i] & GateBit::WARMUP)) match += ((ref[i] ^ b[i]) & GateBit::RAW) == 0;
        worst = std::min(worst, (double)match / valid);
    }
    const double on_frac = (double)ref_on / valid;
    report(worst >= 0.95 && on_frac > 0.2 && on_frac < 0.8, "T6",
           "z-axis tremor, 20 random rotations: min RAW agreement %.3f%% (reference RAW ON %.1f%%)", 100 * worst, 100 * on_frac);
}

// --- T7: incremental ACF vs batch ---------------------------------------------------
double maxAcfError(const TremorGate& g) {
    const int N = g.acfWindow();
    std::vector<Vec3> w(N);
    for (int i = 0; i < N; ++i) w[i] = g.windowSample(i);
    auto dot = [&](int i, int j) {
        return (double)w[i].x * w[j].x + (double)w[i].y * w[j].y + (double)w[i].z * w[j].z;
    };
    double err = 0;
    for (int k = g.lagLo(); k <= g.lagHi(); ++k) {
        double S = 0, E1 = 0, E2 = 0;
        for (int i = k; i < N; ++i) {
            S += dot(i, i - k);
            E1 += dot(i, i);
        }
        for (int i = 0; i < N - k; ++i) E2 += dot(i, i);
        double d = E1 * E2;
        double r = d > 1e-20 ? S / std::sqrt(d) : 0.0;
        r = std::max(-1.0, std::min(1.0, r));
        err = std::max(err, std::fabs(r - (double)g.acfR(k)));
    }
    return err;
}

void testIncrementalAcf() {
    TremorGate g;
    g.setParams(baseParams(5.0f));
    Synth s(71);
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    const long total = (long)(3600 * kFs);
    const int seg = (int)(10 * kFs);
    double worst = 0, worst_last = 0;
    long checks = 0;
    for (long n = 0; n < total; ++n) {
        if (n % seg == 0) {   // scene change every 10 s: quiet / tremor / voluntary / mixed
            int kind = (int)(u(s.rng) * 4);
            s.noise_band_rms = 0.2f + 2.0f * u(s.rng);
            s.tremor_rms = (kind == 1 || kind == 3) ? 1.0f + 19.0f * u(s.rng) : 0.0f;
            s.f_tremor = 4.0f + 6.0f * u(s.rng);
            s.vol_amp = (kind >= 2) ? 20.0f + 80.0f * u(s.rng) : 0.0f;
            s.tdir = s.unitRandom();
            s.vdir = s.unitRandom();
        }
        g.update(s.next());
        if (n < g.warmupSamples()) continue;
        const bool last_minute = n >= total - (long)(60 * kFs);
        if (!last_minute && n % 7 != 0) continue;
        double e = maxAcfError(g);
        ++checks;
        worst = std::max(worst, e);
        if (last_minute) worst_last = std::max(worst_last, e);
    }
    report(worst <= 1e-4, "T7", "1 h, %ld checks: max |r_incr − r_batch| %.2e (last 1 min %.2e, limit 1e-4)", checks, worst,
           worst_last);
}

// --- T8: broadband noise (above θ) -------------------------------------------------
void testBroadbandNoise() {
    const float f0s[] = {5.0f, 8.0f};
    for (float f0 : f0s) {
        TremorGate g;
        g.setParams(baseParams(f0));
        Synth s(81 + (unsigned)f0);
        s.noise_band_rms = 2.0f * kTheta;
        BitStats st = runFor(g, s, 300.0f);
        report(st.frac(st.on) < 0.05, "T8", "f0=%g, broadband noise 2θ 5 min: GATE_ON %.3f%% (S1 %.1f%% S2 %.1f%% S3 %.1f%%)", f0,
               100 * st.frac(st.on), 100 * st.frac(st.s1), 100 * st.frac(st.s2), 100 * st.frac(st.s3));
    }
}

// --- T9: mode logic -----------------------------------------------------------------
void testModes() {
    struct Case { CondMode mode; bool gate, cfg, expect; };
    const Case cases[] = {
        {CondMode::NO_VIB, false, false, false},   {CondMode::NO_VIB, true, true, false},
        {CondMode::CONSTANT, false, false, true},  {CondMode::CONSTANT, true, true, true},
        {CondMode::ADAPTIVE, false, true, false},  {CondMode::ADAPTIVE, true, true, true},
        {CondMode::ADAPTIVE, true, false, false},  {CondMode::ADAPTIVE, false, false, false},
    };
    bool ok = true;
    HapticController hc;
    for (const Case& c : cases) {
        GateOutput o;
        o.bits = (c.gate ? GateBit::GATE_ON : 0) | (c.cfg ? GateBit::CFG_VALID : 0);
        hc.setMode(c.mode);
        ok = ok && hc.decide(o) == c.expect;
    }
    CondMode m;
    ok = ok && !CondMode_fromU8(1, &m) && !CondMode_fromU8(4, &m) && CondMode_fromU8(3, &m) && m == CondMode::ADAPTIVE;

    hc.setMode(CondMode::CONSTANT);
    hc.setTrialActive(true);
    ok = ok && !hc.setMode(CondMode::ADAPTIVE) && hc.mode() == CondMode::CONSTANT;
    hc.setTrialActive(false);
    ok = ok && hc.setMode(CondMode::ADAPTIVE);
    report(ok, "T9", "3 modes × gate/CFG_VALID table, SHAM(1) rejected, mode locked during trial");
}

}  // namespace

int main() {
    testSanity();
    testFilter();
    testLatency();
    testNoiseFalseAlarm();
    testVoluntary();
    testTremorPlusVoluntary();
    testAxisInvariance();
    testIncrementalAcf();
    testBroadbandNoise();
    testModes();
    std::printf("\n%s (%d failed)\n", g_failed ? "FAILED" : "ALL PASSED", g_failed);
    return g_failed ? 1 : 0;
}