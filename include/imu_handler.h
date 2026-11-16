#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdint.h>

// 가속도 [g] + 자이로 [deg/s]. 가속도는 로깅 전용 (게이트에는 자이로만 쓴다).
struct ImuSample {
    float ax, ay, az;
    float gx, gy, gz;
    bool  clipped;   // 어느 축이든 풀스케일에 닿음
};

bool initIMU();          // ODR 104 Hz, 자이로 ±kGyroRangeDps, BDU 설정
bool imuDataReady();     // STATUS_REG의 자이로 data-ready (GDA)
ImuSample readIMU();     // 자이로·가속도 12바이트를 한 번에 읽는다

#endif
