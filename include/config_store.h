#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

// Settings storage on the internal flash (InternalFS / LittleFS), BLE_SPEC §9 (S4).
// Files: /params.bin (last SET_PARAMS) and /trial.bin (in-progress trial). Formats in config_codec.h.
//
// InternalFS FILE_O_WRITE appends, so every save writes a temporary file and renames it over the
// target; LittleFS rename replaces the old file atomically, so a power loss leaves either the old or
// the new file. A corrupt file fails the CRC and is treated as missing.

#include "device_state.h"

class ConfigStore : public DeviceStore {
  public:
    bool begin();

    // false if the file is missing or invalid (length, magic, format, CRC)
    bool loadParams(icd::ParamsWire* p, uint8_t* cfg_ver);
    bool loadTrial(TrialRecord* r);

    bool saveParams(const icd::ParamsWire& p, uint8_t cfg_ver) override;
    bool saveTrial(const TrialRecord& r) override;
    bool clearTrial() override;

  private:
    bool ok_ = false;
};

#endif
