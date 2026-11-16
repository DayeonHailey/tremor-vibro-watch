#include "imu_handler.h"
#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "config.h"

static LSM6DS3 myIMU(I2C_MODE, kImuI2cAddr);

static constexpr uint8_t kStatusGDA = 0x02;
static constexpr uint8_t kCtrl3BDU  = 0x40;   // block data update
static constexpr int16_t kClipRaw   = 32000;

static bool isClipped(int16_t v) { return v >= kClipRaw || v <= -kClipRaw; }

bool initIMU() {
    myIMU.settings.gyroRange       = kGyroRangeDps;
    myIMU.settings.gyroSampleRate  = kImuOdrHz;
    myIMU.settings.accelRange      = kAccelRangeG;
    myIMU.settings.accelSampleRate = kImuOdrHz;
    myIMU.settings.accelBandWidth  = 50;
    if (myIMU.begin() != 0) return false;

    // 상·하위 바이트가 서로 다른 샘플에서 오지 않도록 BDU를 켠다
    uint8_t ctrl3 = 0;
    myIMU.readRegister(&ctrl3, LSM6DS3_ACC_GYRO_CTRL3_C);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, ctrl3 | kCtrl3BDU);
    return true;
}

bool imuDataReady() {
    uint8_t status = 0;
    if (myIMU.readRegister(&status, LSM6DS3_ACC_GYRO_STATUS_REG) != IMU_SUCCESS) return false;
    return (status & kStatusGDA) != 0;
}

ImuSample readIMU() {
    // OUTX_L_G(0x22)부터 자이로 xyz, 가속도 xyz 순서로 연속 12바이트
    uint8_t b[12] = {0};
    myIMU.readRegisterRegion(b, LSM6DS3_ACC_GYRO_OUTX_L_G, sizeof(b));
    int16_t raw[6];
    for (int i = 0; i < 6; ++i) raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));

    ImuSample s;
    s.gx = myIMU.calcGyro(raw[0]);
    s.gy = myIMU.calcGyro(raw[1]);
    s.gz = myIMU.calcGyro(raw[2]);
    s.ax = myIMU.calcAccel(raw[3]);
    s.ay = myIMU.calcAccel(raw[4]);
    s.az = myIMU.calcAccel(raw[5]);
    s.clipped = false;
    for (int i = 0; i < 6; ++i) s.clipped = s.clipped || isClipped(raw[i]);
    return s;
}
