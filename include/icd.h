#ifndef ICD_H
#define ICD_H

// Tremor watch BLE ICD v1.0 (BLE_SPEC §4–§7). Shared reference for firmware and tools/icd.py.
// If you change a value here, update tools/icd.py too; bump kIcdVersion on incompatible changes.
// All multi-byte fields are little-endian (nRF52 native).
// Platform-independent: no Arduino headers.

#include <stdint.h>

namespace icd {

// ICD version (upper 4 bits major, lower 4 bits minor). Bumping the major version also changes the UUID prefix.
constexpr uint8_t kIcdVersion = 0x10;   // 1.0

// --- UUID (§4) ---
// Base 7E1A____-5C3B-4D2E-9F61-8A2B3C4D5E6F. Byte array is LSB first (SoftDevice format).
// [12] = low byte of the underscored part, [13] = high byte.
constexpr uint16_t kUuidService = 0x0100;
constexpr uint16_t kUuidData    = 0x0101;
constexpr uint16_t kUuidCtrl    = 0x0102;
constexpr uint16_t kUuidResp    = 0x0103;
constexpr uint16_t kUuidStatus  = 0x0104;

inline void makeUuid128(uint16_t short_id, uint8_t out[16]) {
    static const uint8_t kBase[16] = {0x6F, 0x5E, 0x4D, 0x3C, 0x2B, 0x8A, 0x61, 0x9F,
                                      0x2E, 0x4D, 0x3B, 0x5C, 0x00, 0x00, 0x1A, 0x7E};
    for (int i = 0; i < 16; ++i) out[i] = kBase[i];
    out[12] = (uint8_t)(short_id & 0xFF);
    out[13] = (uint8_t)(short_id >> 8);
}

constexpr uint16_t kDataMaxLen   = 77;
constexpr uint16_t kCtrlMaxLen   = 40;
constexpr uint16_t kRespMaxLen   = 40;
constexpr uint16_t kStatusMaxLen = 40;
constexpr uint16_t kMinMtu       = 80;   // below this, Data is not sent and err_code = MTU_LOW

// --- State / mode (§8) ---
enum State : uint8_t { STATE_IDLE = 0, STATE_STREAMING = 1, STATE_TRIAL = 2 };

// Data/Status flags
constexpr uint8_t FLAG_GAP_PENDING       = 1u << 0;
constexpr uint8_t FLAG_LOG_ACTIVE        = 1u << 1;
constexpr uint8_t FLAG_LOG_NEAR_FULL     = 1u << 2;
constexpr uint8_t FLAG_CFG_VALID         = 1u << 3;
constexpr uint8_t FLAG_REBOOTED_IN_TRIAL = 1u << 4;

// Status err_code (§7)
enum ErrCode : uint8_t {
    ERR_NONE = 0, ERR_IMU_INIT = 1, ERR_DRV = 2, ERR_QSPI = 3, ERR_MTU_LOW = 4,
};

// --- Data packet (§5) ---
constexpr uint8_t kPktTypeData = 0x01;
constexpr uint8_t kSamplesPerPkt = 4;

struct __attribute__((packed)) DataSample {
    int16_t g[3];   // gyro raw LSB
    int16_t a[3];   // accel raw LSB
    uint8_t bits;   // gate_bits (FIRMWARE_SPEC §4.8)
};
static_assert(sizeof(DataSample) == 13, "DataSample 13 B");

// features scale
constexpr float kScaleA    = 100.0f;     // deg/s
constexpr float kScaleC    = 10000.0f;
constexpr float kScaleR    = 10000.0f;
constexpr float kScaleD    = 10000.0f;
constexpr float kScaleFacf = 100.0f;     // Hz

struct __attribute__((packed)) DataPkt {
    uint8_t    pkt_type;   // kPktTypeData
    uint8_t    n;          // kSamplesPerPkt
    uint32_t   seq0;
    uint32_t   t_us;       // time of the last sample
    uint8_t    state;
    uint8_t    mode;
    uint8_t    cfg_ver;
    uint8_t    trial_id;
    uint8_t    flags;
    DataSample s[kSamplesPerPkt];
    int16_t    feat[5];    // A, C, r, D, f_acf (at the last sample)
};
static_assert(sizeof(DataPkt) == kDataMaxLen, "DataPkt 77 B");

// --- Status (§7) ---
// The spec table's "total 39 B" disagrees with the sum of fields (34 B). Fields take precedence.
struct __attribute__((packed)) StatusPkt {
    uint8_t  state, mode, cfg_ver, trial_id, flags;
    uint8_t  err_code;
    uint16_t battery_mv;
    uint16_t log_free_kb;
    uint32_t seq_now;
    uint32_t gap_seq_start, gap_seq_end;
    uint16_t fifo_overrun, notify_fail, i2c_err, gyro_clip;
    uint16_t conn_interval_x125;
    uint16_t mtu;
};
static_assert(sizeof(StatusPkt) == 34, "StatusPkt 34 B");
static_assert(sizeof(StatusPkt) <= kStatusMaxLen, "Status exceeds max length");

// --- Commands (§6) ---
// Ctrl: [cmd][req_id][args…]   Resp: [req_id][cmd][status][payload…]
enum Cmd : uint8_t {
    CMD_GET_INFO    = 0x01,
    CMD_PING        = 0x02,
    CMD_SET_MODE    = 0x10,
    CMD_SET_PARAMS  = 0x11,
    CMD_GET_PARAMS  = 0x12,
    CMD_TRIAL_START = 0x20,
    CMD_TRIAL_END   = 0x21,
    CMD_STREAM      = 0x22,
    CMD_TEST_PULSE  = 0x30,
    CMD_GATE_RESET  = 0x31,
    CMD_LOG_INFO    = 0x40,
    CMD_LOG_ERASE   = 0x41,
};

enum RespStatus : uint8_t {
    RS_OK              = 0,
    RS_UNKNOWN_CMD     = 1,
    RS_BAD_LEN         = 2,
    RS_OUT_OF_RANGE    = 3,
    RS_LOCKED_IN_TRIAL = 4,
    RS_NOT_CONFIGURED  = 5,
    RS_BUSY            = 6,
    RS_FLASH_ERROR     = 7,
};

constexpr int kRespHeader = 3;

struct __attribute__((packed)) InfoPayload {
    uint8_t  icd_ver;
    uint8_t  fw_ver[3];
    uint32_t build_hash;
    uint64_t device_id;
    uint16_t gyro_fs_dps;
    uint8_t  acc_fs_g;
    uint16_t odr_hz;
    uint8_t  n_per_pkt;
};
static_assert(sizeof(InfoPayload) == 22, "InfoPayload 22 B");
static_assert(kRespHeader + sizeof(InfoPayload) <= kRespMaxLen, "Resp exceeds max length");

struct __attribute__((packed)) PingArgs { uint32_t nonce; };
struct __attribute__((packed)) PingPayload { uint32_t nonce; uint32_t t_us; };

// Feature value → i16 (saturates when out of range)
inline int16_t scaleSat(float v, float scale) {
    float x = v * scale;
    if (!(x == x)) return 0;   // NaN
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

}  // namespace icd

#endif
