#include "config_codec.h"
#include <string.h>

namespace cfgfile {

uint32_t crc32(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static void putU32(uint8_t* o, uint32_t v) {
    o[0] = (uint8_t)v;
    o[1] = (uint8_t)(v >> 8);
    o[2] = (uint8_t)(v >> 16);
    o[3] = (uint8_t)(v >> 24);
}

static uint32_t getU32(const uint8_t* i) {
    return (uint32_t)i[0] | ((uint32_t)i[1] << 8) | ((uint32_t)i[2] << 16) | ((uint32_t)i[3] << 24);
}

// Checks length, magic, format and the trailing CRC
static bool checkFrame(const uint8_t* in, size_t len, size_t want, uint32_t magic) {
    if (len != want) return false;
    if (getU32(in) != magic || in[4] != kFormat) return false;
    return getU32(in + want - 4) == crc32(in, want - 4);
}

void encodeParams(const icd::ParamsWire& p, uint8_t cfg_ver, uint8_t out[kParamsLen]) {
    putU32(out, kMagicParams);
    out[4] = kFormat;
    memcpy(out + 5, &p, sizeof(p));
    out[5 + sizeof(p)] = cfg_ver;
    putU32(out + kParamsLen - 4, crc32(out, kParamsLen - 4));
}

bool decodeParams(const uint8_t* in, size_t len, icd::ParamsWire* p, uint8_t* cfg_ver) {
    if (!checkFrame(in, len, kParamsLen, kMagicParams)) return false;
    memcpy(p, in + 5, sizeof(*p));
    *cfg_ver = in[5 + sizeof(*p)];
    return true;
}

void encodeTrial(const TrialRecord& r, uint8_t out[kTrialLen]) {
    putU32(out, kMagicTrial);
    out[4] = kFormat;
    putU32(out + 5, r.session_id);
    out[9] = r.trial_id;
    out[10] = r.mode;
    putU32(out + kTrialLen - 4, crc32(out, kTrialLen - 4));
}

bool decodeTrial(const uint8_t* in, size_t len, TrialRecord* r) {
    if (!checkFrame(in, len, kTrialLen, kMagicTrial)) return false;
    r->session_id = getU32(in + 5);
    r->trial_id = in[9];
    r->mode = in[10];
    return true;
}

}  // namespace cfgfile
