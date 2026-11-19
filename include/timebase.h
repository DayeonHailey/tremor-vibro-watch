#ifndef TIMEBASE_H
#define TIMEBASE_H

// 샘플 타임스탬프용 µs 시계 (BLE_SPEC §3, §5 t_us).
//
// Arduino micros()는 이 코어에서 DWT가 켜져 있으면 CYCCNT/64 (약 67초마다 넘어감,
// 사이클 계측이 CYCCNT를 건드리면 튐), 꺼져 있으면 FreeRTOS tick (~1 ms 해상도)이라서
// t_us로 쓸 수 없다. 대신 RTC2(32.768 kHz 크리스털, ±20 ppm)를 64비트로 확장해 쓴다.
//   해상도 30.5 µs, 반환값은 2^32 µs(약 71.6분)마다 넘어간다 → PC가 이어 붙인다.
// RTC0은 SoftDevice, RTC1은 FreeRTOS가 쓰므로 RTC2만 쓴다.
// ISR 안에서도 호출할 수 있다.

#include <stdint.h>

void     timebase_init();
uint32_t timebase_us();

#endif
