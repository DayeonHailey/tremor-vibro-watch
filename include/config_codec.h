#ifndef CONFIG_CODEC_H
#define CONFIG_CODEC_H

// File formats for /params.bin and /trial.bin (BLE_SPEC §9). Platform-independent so host tests can
// check encode/decode and CRC handling; file I/O is in config_store.
//
// /params.bin: magic u32, format u8, ParamsWire (36 B), cfg_ver u8, CRC32 u32   = 46 B
// /trial.bin : magic u32, format u8, session_id u32, trial_id u8, mode u8, CRC32 u32 = 15 B
// CRC32 (IEEE 802.3, reflected 0xEDB88320) covers every byte before it. Little-endian.

#include <stddef.h>
#include <stdint.h>
#include "icd.h"
#include "device_state.h"

namespace cfgfile {

constexpr uint32_t kMagicParams = 0x50574D54;   // "TMWP"
constexpr uint32_t kMagicTrial  = 0x54574D54;   // "TMWT"
constexpr uint8_t  kFormat      = 1;
constexpr size_t   kParamsLen   = 4 + 1 + sizeof(icd::ParamsWire) + 1 + 4;
constexpr size_t   kTrialLen    = 4 + 1 + 4 + 1 + 1 + 4;

uint32_t crc32(const uint8_t* p, size_t n);

void encodeParams(const icd::ParamsWire& p, uint8_t cfg_ver, uint8_t out[kParamsLen]);
// false on wrong length, magic, format or CRC
bool decodeParams(const uint8_t* in, size_t len, icd::ParamsWire* p, uint8_t* cfg_ver);

void encodeTrial(const TrialRecord& r, uint8_t out[kTrialLen]);
bool decodeTrial(const uint8_t* in, size_t len, TrialRecord* r);

}  // namespace cfgfile

#endif
