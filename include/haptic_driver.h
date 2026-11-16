#ifndef HAPTIC_DRIVER_H
#define HAPTIC_DRIVER_H

// DRV2605L 구동 (FIRMWARE_SPEC §6). 판단 로직 없음: on/off만 받는다.

void Haptic_Init();
// 샘플마다 호출. on이면 RTP kVibLevel, off가 되면 RTP 0 → kBrakeMs 후 standby.
// 상태가 바뀔 때만 I2C를 쓴다.
void Haptic_Drive(bool on);
// 모터를 실제로 구동 중인지 (VIB_ON 비트)
bool Haptic_IsActive();

#endif
