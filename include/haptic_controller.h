#ifndef HAPTIC_CONTROLLER_H
#define HAPTIC_CONTROLLER_H

// 조건 모드 × 게이트 → 모터 on/off (FIRMWARE_SPEC §6). 플랫폼 독립.
// 진동 레벨은 config.h의 kVibLevel 하나로 고정이고, 여기서는 on/off만 정한다.

#include <stdint.h>
#include "tremor_gate.h"

// 번호는 실험 로그·GATT 값과 묶여 있으므로 바꾸지 않는다.
enum class CondMode : uint8_t {
    NO_VIB   = 0,
    /* 1 = SHAM (예약, 구현 안 함) */
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
    // trial 진행 중에는 모드 전환을 거부한다 (trial 경계에서만 허용).
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
