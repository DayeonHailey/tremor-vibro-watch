// I2C 스캐너 (진단용). 빌드에 포함되지 않는다.
// 쓰려면 src/main.cpp 대신 이 파일을 src/에 두고 업로드한다.
#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    pinMode(D1, OUTPUT);
    digitalWrite(D1, HIGH);   // DRV EN — 스캔 전에 반드시 깨워둠
    delay(10);

    Wire.begin();
    Wire.setClock(100000);    // 100kHz 명시
    delay(50);

    Serial.println("Scanning...");

    Serial.print("D1 read-back: ");
    Serial.println(digitalRead(D1));  // HIGH면 1, 아니면 0
}

void loop() {
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (!found) Serial.println("No I2C devices.");
    delay(2000);
}
