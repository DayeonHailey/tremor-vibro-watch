#ifndef HAPTIC_CONTROLLER_H
#define HAPTIC_CONTROLLER_H

// Condition mode × gate → motor on/off (FIRMWARE_SPEC §6). Platform-independent.
// Vibration level is fixed by kVibLevel in config.h; this only decides on/off.

#include <stdint.h>
#include "tremor_gate.h"

// Values are tied to experiment logs and GATT values; do not renumber.
enum class CondMode : uint8_t {
    NO_VIB   = 0,
    /* 1 = SHAM (reserved, not implemented) */
    CONSTANT = 2,
    ADAPTIVE = 3,
};

inline bool CondMode_fromU8(uint8_t v, CondMode* out) {
    if (v != 0 && v != 2 && v != 3) return false;
    *out = static_cast<CondMode>(v);
    return true;
}

class HapticController {
  public:
    // Mode changes are rejected during a trial (allowed only at trial boundaries).
    bool setMode(CondMode m) {
        if (trial_active_) return false;
        mode_ = m;
        return true;
    }
    CondMode mode() const { return mode_; }

    void setTrialActive(bool active) { trial_active_ = active; }
    bool trialActive() const { return trial_active_; }

    bool decide(const GateOutput& g) const {
        switch (mode_) {
            case CondMode::CONSTANT:
                return true;
            case CondMode::ADAPTIVE: {
                const uint8_t need = GateBit::GATE_ON | GateBit::CFG_VALID;
                return (g.bits & need) == need;
            }
            case CondMode::NO_VIB:
            default:
                return false;
        }
    }

  private:
    CondMode mode_ = CondMode::NO_VIB;
    bool trial_active_ = false;
};

#endif
