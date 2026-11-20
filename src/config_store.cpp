#include "config_store.h"
#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "config_codec.h"
#include "haptic_driver.h"

using namespace Adafruit_LittleFS_Namespace;

static const char kParamsPath[] = "/params.bin";
static const char kParamsTmp[]  = "/params.tmp";
static const char kTrialPath[]  = "/trial.bin";
static const char kTrialTmp[]   = "/trial.tmp";

static bool writeAtomic(const char* path, const char* tmp, const uint8_t* data, size_t len) {
    if (InternalFS.exists(tmp)) InternalFS.remove(tmp);
    File f = InternalFS.open(tmp, FILE_O_WRITE);
    if (!f) return false;
    const size_t n = f.write(data, len);
    f.close();
    if (n != len) {
        InternalFS.remove(tmp);
        return false;
    }
    return InternalFS.rename(tmp, path);
}

// Reads the whole file into buf; returns the byte count, or 0 if missing/too large
static size_t readAll(const char* path, uint8_t* buf, size_t cap) {
    if (!InternalFS.exists(path)) return 0;
    File f = InternalFS.open(path, FILE_O_READ);
    if (!f) return 0;
    const size_t size = f.size();
    size_t n = 0;
    if (size <= cap) n = f.read(buf, size);
    f.close();
    return (n == size) ? n : 0;
}

bool ConfigStore::begin() {
    ok_ = InternalFS.begin();
    return ok_;
}

bool ConfigStore::loadParams(icd::ParamsWire* p, uint8_t* cfg_ver) {
    if (!ok_) return false;
    uint8_t buf[cfgfile::kParamsLen + 1];   // +1 so an oversized file is detected
    const size_t n = readAll(kParamsPath, buf, sizeof(buf));
    return cfgfile::decodeParams(buf, n, p, cfg_ver);
}

bool ConfigStore::loadTrial(TrialRecord* r) {
    if (!ok_) return false;
    uint8_t buf[cfgfile::kTrialLen + 1];
    const size_t n = readAll(kTrialPath, buf, sizeof(buf));
    return cfgfile::decodeTrial(buf, n, r);
}

bool ConfigStore::saveParams(const icd::ParamsWire& p, uint8_t cfg_ver) {
    if (!ok_) return false;
    uint8_t buf[cfgfile::kParamsLen];
    cfgfile::encodeParams(p, cfg_ver, buf);
    return writeAtomic(kParamsPath, kParamsTmp, buf, sizeof(buf));
}

bool ConfigStore::saveTrial(const TrialRecord& r) {
    if (!ok_) return false;
    uint8_t buf[cfgfile::kTrialLen];
    cfgfile::encodeTrial(r, buf);
    return writeAtomic(kTrialPath, kTrialTmp, buf, sizeof(buf));
}

bool ConfigStore::clearTrial() {
    // Called from TRIAL_END after the trial state is already off. The flash delete below blocks loop()
    // for >1 s, so stop the motor first instead of letting it run until loop() resumes.
    Haptic_Drive(false);
    if (!ok_) return false;
    return !InternalFS.exists(kTrialPath) || InternalFS.remove(kTrialPath);
}
