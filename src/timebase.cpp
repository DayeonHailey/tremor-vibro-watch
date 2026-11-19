#include "timebase.h"
#include <Arduino.h>

static volatile uint32_t rtc_ovf = 0;   // number of 24-bit counter overflows (+1 every 512 s)

extern "C" void RTC2_IRQHandler(void) {
    if (NRF_RTC2->EVENTS_OVRFLW) {
        NRF_RTC2->EVENTS_OVRFLW = 0;
        (void)NRF_RTC2->EVENTS_OVRFLW;   // wait for the write to take effect to avoid IRQ re-entry
        rtc_ovf = rtc_ovf + 1;
    }
}

void timebase_init() {
    // LFCLK (crystal) is already running from core init (wiring.c)
    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->TASKS_CLEAR = 1;
    NRF_RTC2->PRESCALER = 0;   // 32768 Hz
    NRF_RTC2->EVENTS_OVRFLW = 0;
    NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    NVIC_SetPriority(RTC2_IRQn, 6);   // application priority allowed by the SoftDevice
    NVIC_ClearPendingIRQ(RTC2_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);
    NRF_RTC2->TASKS_START = 1;
}

uint32_t timebase_us() {
    uint32_t ovf, cnt;
    bool pending;
    do {
        ovf = rtc_ovf;
        cnt = NRF_RTC2->COUNTER;
        pending = NRF_RTC2->EVENTS_OVRFLW != 0;
    } while (ovf != rtc_ovf);
    // Overflowed but the ISR has not run yet (e.g. called from a higher-priority ISR)
    if (pending && cnt < 0x800000u) ++ovf;

    const uint64_t ticks = ((uint64_t)ovf << 24) | cnt;
    return (uint32_t)((ticks * 15625u) >> 9);   // × 10^6 / 32768
}
