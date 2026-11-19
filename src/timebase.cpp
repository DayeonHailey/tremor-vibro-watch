#include "timebase.h"
#include <Arduino.h>

static volatile uint32_t rtc_ovf = 0;   // 24비트 카운터가 넘어간 횟수 (512초마다 +1)

extern "C" void RTC2_IRQHandler(void) {
    if (NRF_RTC2->EVENTS_OVRFLW) {
        NRF_RTC2->EVENTS_OVRFLW = 0;
        (void)NRF_RTC2->EVENTS_OVRFLW;   // 쓰기 반영을 기다려 IRQ 재진입 방지
        rtc_ovf = rtc_ovf + 1;
    }
}

void timebase_init() {
    // LFCLK(크리스털)은 코어 초기화(wiring.c)에서 이미 켜져 있다
    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->TASKS_CLEAR = 1;
    NRF_RTC2->PRESCALER = 0;   // 32768 Hz
    NRF_RTC2->EVENTS_OVRFLW = 0;
    NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    NVIC_SetPriority(RTC2_IRQn, 6);   // SoftDevice가 허용하는 앱 우선순위
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
    // 넘어갔지만 ISR이 아직 못 돈 경우 (더 높은 우선순위 ISR 안에서 호출 등)
    if (pending && cnt < 0x800000u) ++ovf;

    const uint64_t ticks = ((uint64_t)ovf << 24) | cnt;
    return (uint32_t)((ticks * 15625u) >> 9);   // × 10^6 / 32768
}
