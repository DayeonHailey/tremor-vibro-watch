// I2C scanner (diagnostic). Not part of the build.
// To use it, put this file in src/ instead of src/main.cpp and upload.
#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    pinMode(D1, OUTPUT);
    digitalWrite(D1, HIGH);   // DRV EN — must be awake before scanning
    delay(10);

    Wire.begin();
    Wire.setClock(100000);    // explicitly 100 kHz
    delay(50);

    Serial.println("Scanning...");

    Serial.print("D1 read-back: ");
    Serial.println(digitalRead(D1));  // 1 if HIGH, else 0
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
