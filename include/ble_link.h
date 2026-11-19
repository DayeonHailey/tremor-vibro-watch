#ifndef BLE_LINK_H
#define BLE_LINK_H

// Bluefruit GATT server (BLE_SPEC §4). Transport only; does not interpret commands.
//   Ctrl write → only queued (loop pops and handles it via popCtrl, §2 rule).
//   Exception: PING is answered directly in the BLE callback to minimize latency (§6).

#include <stdint.h>
#include "icd.h"

namespace ble {

struct CtrlMsg {
    uint8_t len;
    uint8_t data[icd::kCtrlMaxLen];
};

void begin(uint64_t device_id);

bool connected();
bool dataSubscribed();          // PC has enabled Data notifications
uint16_t mtu();                 // 0 when not connected
uint16_t connIntervalX125();    // connection interval ÷ 1.25 ms, 0 when not connected
uint16_t notifyFail();          // cumulative Data notify failures

bool popCtrl(CtrlMsg* out);     // call from loop
bool sendData(const icd::DataPkt& pkt);
void sendStatus(const icd::StatusPkt& st);   // update Read value + notify if subscribed
void sendResp(uint8_t req_id, uint8_t cmd, uint8_t status, const void* payload = nullptr,
              uint8_t len = 0);

}  // namespace ble

#endif
