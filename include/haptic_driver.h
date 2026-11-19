#ifndef HAPTIC_DRIVER_H
#define HAPTIC_DRIVER_H

// DRV2605L driver (FIRMWARE_SPEC §6). No decision logic: takes on/off only.

void Haptic_Init();
// Call every sample. on → RTP kVibLevel; on → off → RTP 0, then standby after kBrakeMs.
// Writes I2C only when the state changes.
void Haptic_Drive(bool on);
// Whether the motor is actually being driven (VIB_ON bit)
bool Haptic_IsActive();

#endif
