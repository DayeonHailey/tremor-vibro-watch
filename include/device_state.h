#ifndef DEVICE_STATE_H
#define DEVICE_STATE_H

// Device state machine and command handlers (BLE_SPEC §6, §8, S3).
// Platform-independent (no Arduino): time is passed in as now_ms, so host tests can drive it.
// Transport-independent: BLE Ctrl (via dispatch) and USB Serial call the same cmd* functions.
//
// States: IDLE (no connection or STREAM off) / STREAMING / TRIAL.
// Safety rule: the motor runs only in TRIAL (per mode). TEST_PULSE is the only exception and
// is rejected during TRIAL. In every other state the motor is off regardless of mode.
//
// Gap tracking (§8): if the link drops during TRIAL, the trial continues. The gap is the half-open
// seq range [gap_seq_start, gap_seq_end): start = first seq not delivered before the drop,
// end = next seq when the PC re-subscribes to Data. GAP_PENDING stays set until the next STREAM command.

#include <stddef.h>
#include <stdint.h>
#include "icd.h"
#include "tremor_gate.h"
#include "haptic_controller.h"

// In-progress trial record (§8): written at TRIAL_START, deleted at TRIAL_END, used to resume after a reboot.
struct TrialRecord {
    uint32_t session_id;
    uint8_t  trial_id;
    uint8_t  mode;
};

// Persistent storage used by Device (§9). Implemented with InternalFS on the watch, faked in host tests.
class DeviceStore {
  public:
    virtual ~DeviceStore() {}
    virtual bool saveParams(const icd::ParamsWire& p, uint8_t cfg_ver) = 0;
    virtual bool saveTrial(const TrialRecord& r) = 0;
    virtual bool clearTrial() = 0;
};

class Device {
  public:
    // Link state, reported every loop by main. Edges drive the state machine.
    struct Link {
        bool     connected;
        bool     data_subscribed;
        bool     mtu_ok;
        uint32_t seq_next;   // seq of the next sample to be read (imuNextSeq)
    };

    void setInfo(const icd::InfoPayload& info) { info_ = info; }
    void setStore(DeviceStore* s) { store_ = s; }
    void updateLink(const Link& l);

    // --- Boot-time restore (§9). Nothing is written back to the store. ---
    // Applies stored parameters with the same checks as SET_PARAMS. false → left unconfigured.
    bool restoreParams(const icd::ParamsWire& p, uint8_t cfg_ver);
    // Re-enters TRIAL with the same trial_id and mode, sets REBOOTED_IN_TRIAL. Call after restoreParams.
    // false if the record is unusable (e.g. ADAPTIVE without CFG_VALID); the caller should clear it.
    bool resumeTrial(const TrialRecord& r);

    TremorGate& gate() { return gate_; }
    const HapticController& haptic() const { return hctrl_; }

    // --- Outputs ---
    uint8_t  state() const;
    uint8_t  flags() const;
    uint8_t  mode() const { return (uint8_t)hctrl_.mode(); }
    uint8_t  cfgVer() const { return cfg_ver_; }
    uint8_t  trialId() const { return trial_id_; }
    uint32_t sessionId() const { return session_id_; }
    bool     cfgValid() const { return gate_.cfgValid(); }
    uint32_t gapStart() const { return gap_start_; }
    uint32_t gapEnd() const { return gap_end_; }

    bool shouldSendData() const;
    void onDataSent(uint32_t seq0, uint8_t n);   // call only when the notify succeeded
    bool motorOn(const GateOutput& g, uint32_t now_ms) const;
    bool takeStatusDirty();                      // true once after a state/flag change

    // --- Commands (return icd::RespStatus) ---
    uint8_t cmdSetMode(uint8_t m);
    uint8_t cmdSetParams(const icd::ParamsWire& w, uint8_t* cfg_ver_out);
    void    cmdGetParams(icd::GetParamsPayload* out) const;
    uint8_t cmdTrialStart(uint8_t trial_id);
    uint8_t cmdTrialEnd(uint8_t trial_id);
    uint8_t cmdStream(uint8_t on);
    uint8_t cmdTestPulse(uint16_t ms, uint32_t now_ms);
    uint8_t cmdGateReset();
    uint8_t cmdSetParamField(const char* name, float value);   // Serial-only convenience

    // Binary Ctrl frame [cmd][req_id][args…] → status + payload. PING is handled in the BLE callback.
    // payload must hold kRespMaxLen - kRespHeader bytes.
    uint8_t dispatch(const uint8_t* req, size_t len, uint8_t* payload, uint8_t* payload_len, uint32_t now_ms);

  private:
    bool applyParams(const icd::ParamsWire& w);   // §6.1 checks + apply to the gate

    DeviceStore*     store_ = nullptr;
    bool             rebooted_in_trial_ = false;
    TremorGate       gate_;
    HapticController hctrl_;
    icd::InfoPayload info_ = {};

    uint8_t  cfg_ver_ = 0;
    uint32_t session_id_ = 0;
    bool     stream_on_ = false;
    bool     trial_ = false;
    uint8_t  trial_id_ = 0;

    uint32_t pulse_end_ms_ = 0;
    bool     pulse_on_ = false;

    Link     link_ = {false, false, false, 0};
    uint32_t last_sent_end_ = 0;   // seq following the last delivered packet
    bool     sent_any_ = false;
    bool     gap_open_ = false;    // link dropped during TRIAL, not yet re-subscribed
    bool     gap_pending_ = false;
    uint32_t gap_start_ = 0, gap_end_ = 0;
    bool     status_dirty_ = false;
};

#endif
