#ifndef TIMEBASE_H
#define TIMEBASE_H

// Microsecond clock for sample timestamps (BLE_SPEC §3, §5 t_us).
//
// Arduino micros() on this core is CYCCNT/64 when DWT is enabled (wraps every ~67 s and
// jumps if cycle profiling touches CYCCNT), otherwise the FreeRTOS tick (~1 ms resolution),
// so it cannot be used for t_us. Instead, RTC2 (32.768 kHz crystal, ±20 ppm) is extended to 64 bits.
//   Resolution 30.5 µs; the return value wraps every 2^32 µs (~71.6 min) → the PC unwraps it.
// RTC0 is used by the SoftDevice and RTC1 by FreeRTOS, so only RTC2 is used here.
// Safe to call from an ISR.

#include <stdint.h>

void     timebase_init();
uint32_t timebase_us();

#endif
