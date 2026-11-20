#include "device_state.h"
#include <string.h>

using namespace icd;

static bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }   // rejects NaN

void Device::updateLink(const Link& l) {
    const Link prev = link_;
    link_ = l;

    if (prev.connected && !l.connected) {
        pulse_on_ = false;
        if (trial_) {
            // Trial continues (vibration, gate, logging). Remember where delivery stopped.
            if (!gap_open_) {
                gap_open_ = true;
                gap_start_ = sent_any_ ? last_sent_end_ : l.seq_next;
                gap_end_ = gap_start_;
            }
        } else {
            stream_on_ = false;   // reconnect needs a new STREAM command
        }
        status_dirty_ = true;
    }

    // Re-subscribed during TRIAL: delivery resumes with the next batch
    if (gap_open_ && l.connected && l.data_subscribed && !prev.data_subscribed) {
        gap_open_ = false;
        gap_end_ = l.seq_next;
        gap_pending_ = true;
        status_dirty_ = true;
    }

    if (prev.connected != l.connected || prev.data_subscribed != l.data_subscribed) status_dirty_ = true;
}

uint8_t Device::state() const {
    if (trial_) return STATE_TRIAL;
    return (link_.connected && stream_on_) ? STATE_STREAMING : STATE_IDLE;
}

uint8_t Device::flags() const {
    uint8_t f = 0;
    if (gap_pending_) f |= FLAG_GAP_PENDING;
    if (cfgValid()) f |= FLAG_CFG_VALID;
    if (rebooted_in_trial_) f |= FLAG_REBOOTED_IN_TRIAL;
    // LOG_ACTIVE / LOG_NEAR_FULL: S5
    return f;
}

bool Device::shouldSendData() const {
    if (!link_.connected || !link_.data_subscribed || !link_.mtu_ok) return false;
    // In TRIAL, re-subscribing alone resumes delivery (§8); otherwise STREAM on is required
    return trial_ || stream_on_;
}

void Device::onDataSent(uint32_t seq0, uint8_t n) {
    last_sent_end_ = seq0 + n;
    sent_any_ = true;
}

bool Device::motorOn(const GateOutput& g, uint32_t now_ms) const {
    if (trial_) return hctrl_.decide(g);
    return pulse_on_ && (int32_t)(pulse_end_ms_ - now_ms) > 0;
}

bool Device::takeStatusDirty() {
    const bool d = status_dirty_;
    status_dirty_ = false;
    return d;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
uint8_t Device::cmdSetMode(uint8_t m) {
    if (trial_) return RS_LOCKED_IN_TRIAL;
    CondMode cm;
    if (!CondMode_fromU8(m, &cm)) return RS_OUT_OF_RANGE;
    hctrl_.setMode(cm);
    status_dirty_ = true;
    return RS_OK;
}

bool Device::applyParams(const ParamsWire& w) {
    // §6.1 ranges. Any violation rejects the whole set (no partial apply).
    const bool ok = w.session_id != 0 &&
                    inRange(w.f0, 3.5f, 12.0f) &&
                    inRange(w.theta_on, 0.05f, 200.0f) &&
                    inRange(w.C_on, 0.0f, 1.0f) && inRange(w.C_off, 0.0f, 1.0f) && w.C_off < w.C_on &&
                    inRange(w.r_on, 0.0f, 1.0f) && inRange(w.r_off, 0.0f, 1.0f) && w.r_off < w.r_on &&
                    inRange(w.D_on, 0.0f, 2.0f) && inRange(w.D_off, 0.0f, 2.0f) && w.D_off < w.D_on;
    if (!ok) return false;

    GateParams p = gate_.params();
    p.f0 = w.f0;
    p.theta_on = w.theta_on;
    p.theta_set = true;
    p.C_on = w.C_on;
    p.C_off = w.C_off;
    p.r_on = w.r_on;
    p.r_off = w.r_off;
    p.D_on = w.D_on;
    p.D_off = w.D_off;
    // Recomputes filter coefficients and re-enters warmup
    if (!gate_.setParams(p)) return false;
    session_id_ = w.session_id;
    status_dirty_ = true;
    return true;
}

uint8_t Device::cmdSetParams(const ParamsWire& w, uint8_t* cfg_ver_out) {
    if (trial_) return RS_LOCKED_IN_TRIAL;
    if (!applyParams(w)) return RS_OUT_OF_RANGE;
    ++cfg_ver_;
    if (cfg_ver_out) *cfg_ver_out = cfg_ver_;
    // Applied even if the save fails; the PC retries SET_PARAMS on FLASH_ERROR (§9: GET_PARAMS compare on connect).
    if (store_ && !store_->saveParams(w, cfg_ver_)) return RS_FLASH_ERROR;
    return RS_OK;
}

bool Device::restoreParams(const ParamsWire& p, uint8_t cfg_ver) {
    if (!applyParams(p)) return false;
    cfg_ver_ = cfg_ver;
    return true;
}

bool Device::resumeTrial(const TrialRecord& r) {
    CondMode m;
    if (r.trial_id < kTrialIdMin || !CondMode_fromU8(r.mode, &m)) return false;
    if (m == CondMode::ADAPTIVE && !cfgValid()) return false;
    hctrl_.setTrialActive(false);
    hctrl_.setMode(m);
    hctrl_.setTrialActive(true);
    trial_ = true;
    trial_id_ = r.trial_id;
    rebooted_in_trial_ = true;
    // seq restarted at 0 on boot and nothing has been delivered since: the gap runs from 0 until re-subscribe
    gap_open_ = true;
    gap_start_ = gap_end_ = 0;
    status_dirty_ = true;
    return true;
}

void Device::cmdGetParams(GetParamsPayload* out) const {
    const GateParams& p = gate_.params();
    out->p.session_id = session_id_;
    out->p.f0 = p.f0;
    out->p.theta_on = p.theta_on;
    out->p.C_on = p.C_on;
    out->p.C_off = p.C_off;
    out->p.r_on = p.r_on;
    out->p.r_off = p.r_off;
    out->p.D_on = p.D_on;
    out->p.D_off = p.D_off;
    out->cfg_ver = cfg_ver_;
    out->valid = cfgValid() ? 1 : 0;
}

uint8_t Device::cmdTrialStart(uint8_t trial_id) {
    if (trial_) return RS_LOCKED_IN_TRIAL;
    if (trial_id < kTrialIdMin) return RS_OUT_OF_RANGE;
    if (hctrl_.mode() == CondMode::ADAPTIVE && !cfgValid()) return RS_NOT_CONFIGURED;
    // S5: reject with RS_BUSY if the backup log is full; start a log segment.
    // Without the record a reboot could not resume this trial, so do not start it.
    if (store_ && !store_->saveTrial(TrialRecord{session_id_, trial_id, mode()})) return RS_FLASH_ERROR;
    trial_ = true;
    trial_id_ = trial_id;
    pulse_on_ = false;
    hctrl_.setTrialActive(true);
    status_dirty_ = true;
    return RS_OK;
}

uint8_t Device::cmdTrialEnd(uint8_t trial_id) {
    if (!trial_ || trial_id != trial_id_) return RS_OUT_OF_RANGE;
    // S5: close the log segment.
    // The trial ends (motor stops) even if the record cannot be deleted; FLASH_ERROR tells the PC that a
    // reboot would resume it.
    trial_ = false;
    trial_id_ = 0;
    rebooted_in_trial_ = false;
    hctrl_.setTrialActive(false);
    status_dirty_ = true;
    if (store_ && !store_->clearTrial()) return RS_FLASH_ERROR;
    return RS_OK;
}

uint8_t Device::cmdStream(uint8_t on) {
    if (on > 1) return RS_OUT_OF_RANGE;
    stream_on_ = (on == 1);
    // The PC has read the gap from Status by now; a still-open gap is kept
    if (!gap_open_) gap_pending_ = false;
    status_dirty_ = true;
    return RS_OK;
}

uint8_t Device::cmdTestPulse(uint16_t ms, uint32_t now_ms) {
    if (trial_) return RS_LOCKED_IN_TRIAL;
    if (ms < kTestPulseMinMs || ms > kTestPulseMaxMs) return RS_OUT_OF_RANGE;
    pulse_on_ = true;
    pulse_end_ms_ = now_ms + ms;
    return RS_OK;
}

uint8_t Device::cmdGateReset() {
    gate_.reset();
    return RS_OK;
}

uint8_t Device::cmdSetParamField(const char* name, float value) {
    if (trial_) return RS_LOCKED_IN_TRIAL;
    GateParams p = gate_.params();
    if (!GateParams_set(p, name, value)) return RS_OUT_OF_RANGE;
    if (!gate_.setParams(p)) return RS_OUT_OF_RANGE;
    ++cfg_ver_;
    status_dirty_ = true;
    return RS_OK;
}

// ---------------------------------------------------------------------------
// Binary dispatch (BLE Ctrl)
// ---------------------------------------------------------------------------
template <typename T>
static T readArg(const uint8_t* p) {
    T v;
    memcpy(&v, p, sizeof(T));
    return v;
}

uint8_t Device::dispatch(const uint8_t* req, size_t len, uint8_t* payload, uint8_t* payload_len,
                         uint32_t now_ms) {
    *payload_len = 0;
    if (len < 2) return RS_BAD_LEN;
    const uint8_t cmd = req[0];
    const uint8_t* a = req + 2;
    const size_t n = len - 2;

    switch (cmd) {
        case CMD_GET_INFO:
            if (n != 0) return RS_BAD_LEN;
            memcpy(payload, &info_, sizeof(info_));
            *payload_len = sizeof(info_);
            return RS_OK;

        case CMD_SET_MODE:
            if (n != 1) return RS_BAD_LEN;
            return cmdSetMode(a[0]);

        case CMD_SET_PARAMS: {
            if (n != sizeof(ParamsWire)) return RS_BAD_LEN;
            uint8_t v = 0;
            const uint8_t st = cmdSetParams(readArg<ParamsWire>(a), &v);
            if (st == RS_OK) {
                payload[0] = v;
                *payload_len = 1;
            }
            return st;
        }

        case CMD_GET_PARAMS: {
            if (n != 0) return RS_BAD_LEN;
            GetParamsPayload gp;
            cmdGetParams(&gp);
            memcpy(payload, &gp, sizeof(gp));
            *payload_len = sizeof(gp);
            return RS_OK;
        }

        case CMD_TRIAL_START:
            if (n != 1) return RS_BAD_LEN;
            return cmdTrialStart(a[0]);

        case CMD_TRIAL_END:
            if (n != 1) return RS_BAD_LEN;
            return cmdTrialEnd(a[0]);

        case CMD_STREAM:
            if (n != 1) return RS_BAD_LEN;
            return cmdStream(a[0]);

        case CMD_TEST_PULSE:
            if (n != 2) return RS_BAD_LEN;
            return cmdTestPulse(readArg<uint16_t>(a), now_ms);

        case CMD_GATE_RESET:
            if (n != 0) return RS_BAD_LEN;
            return cmdGateReset();

        // CMD_LOG_INFO / CMD_LOG_ERASE: S5 (backup log)
        default:
            return RS_UNKNOWN_CMD;
    }
}
