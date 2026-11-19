// Feed recorded CSV data through the gate and print the results (FIRMWARE_SPEC §9).
//
//   replay <in.csv> [field=value ...] > out.csv
//   e.g. replay session01.csv f0=5.2 theta_on=3.1 C_on=0.6
//
// Input: CSV whose header row has gx, gy, gz columns (firmware Serial output works as-is; '#' lines are ignored).
//        If a seq column exists it is copied through; otherwise the row number is used.
// Output: seq,bits,A,C,r,D,f_acf (stdout) / per-bit ON ratio summary (stderr)
// Parameters are validated with the same GateParams_set rules as the firmware.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "tremor_gate.h"

static std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        size_t c = line.find(',', start);
        out.push_back(line.substr(start, c == std::string::npos ? std::string::npos : c - start));
        if (c == std::string::npos) break;
        start = c + 1;
    }
    for (std::string& s : out) {
        while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
        while (!s.empty() && s.front() == ' ') s.erase(0, 1);
    }
    return out;
}

static int findCol(const std::vector<std::string>& h, const char* name) {
    for (size_t i = 0; i < h.size(); ++i)
        if (h[i] == name) return (int)i;
    return -1;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: replay <in.csv> [field=value ...]\n");
        return 2;
    }

    GateParams p;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        size_t eq = arg.find('=');
        if (eq == std::string::npos || !GateParams_set(p, arg.substr(0, eq).c_str(), std::strtof(arg.c_str() + eq + 1, nullptr))) {
            std::fprintf(stderr, "invalid parameter: %s\n", argv[i]);
            return 2;
        }
    }
    TremorGate gate;
    gate.setParams(p);
    if (!gate.cfgValid()) std::fprintf(stderr, "warning: f0/theta_on not set → CFG_VALID=0, RAW and GATE_ON always 0\n");

    FILE* f = std::fopen(argv[1], "r");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 2;
    }

    std::vector<std::string> header;
    int cx = -1, cy = -1, cz = -1, cseq = -1;
    long row = 0, bad = 0;
    long cnt[8] = {0};
    char buf[1024];
    std::printf("seq,bits,A,C,r,D,f_acf\n");
    while (std::fgets(buf, sizeof(buf), f)) {
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cols = split(line);
        if (cx < 0) {
            header = cols;
            cx = findCol(header, "gx");
            cy = findCol(header, "gy");
            cz = findCol(header, "gz");
            cseq = findCol(header, "seq");
            if (cx < 0 || cy < 0 || cz < 0) {
                std::fprintf(stderr, "header must contain gx, gy, gz\n");
                return 2;
            }
            continue;
        }
        if ((int)cols.size() != (int)header.size()) {   // truncated line, repeated header, etc.
            ++bad;
            continue;
        }
        char* end;
        Vec3 g;
        g.x = std::strtof(cols[cx].c_str(), &end);
        g.y = std::strtof(cols[cy].c_str(), &end);
        g.z = std::strtof(cols[cz].c_str(), &end);
        const GateOutput& o = gate.update(g);
        for (int b = 0; b < 8; ++b) cnt[b] += (o.bits >> b) & 1;
        std::printf("%s,%u,%.4f,%.4f,%.4f,%.4f,%.3f\n", cseq >= 0 ? cols[cseq].c_str() : std::to_string(row).c_str(),
                    o.bits, o.A, o.C, o.r, o.D, o.f_acf);
        ++row;
    }
    std::fclose(f);

    static const char* const names[8] = {"S1", "S2", "S3", "RAW", "GATE_ON", "VIB_ON", "CFG_VALID", "WARMUP"};
    std::fprintf(stderr, "%ld samples (%.1f s), %ld skipped lines\n", row, row / p.fs, bad);
    for (int b = 0; b < 8; ++b) {
        if (b == 5) continue;   // VIB_ON is a firmware-side bit
        std::fprintf(stderr, "  %-9s %6.2f%%\n", names[b], row ? 100.0 * cnt[b] / row : 0.0);
    }
    return 0;
}
