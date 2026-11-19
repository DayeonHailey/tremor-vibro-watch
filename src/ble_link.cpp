#include "ble_link.h"
#include <Arduino.h>
#include <bluefruit.h>
#include <string.h>
#include "config.h"
#include "timebase.h"

namespace ble {

static BLEDfu  bledfu;
static BLEDis  bledis;
static BLEBas  blebas;

static uint8_t uuid_svc[16], uuid_data[16], uuid_ctrl[16], uuid_resp[16], uuid_status[16];
static BLEService*        svc;
static BLECharacteristic* chr_data;
static BLECharacteristic* chr_ctrl;
static BLECharacteristic* chr_resp;
static BLECharacteristic* chr_status;

static volatile uint16_t conn_hdl = BLE_CONN_HANDLE_INVALID;
static uint16_t notify_fail = 0;

// Ctrl queue: single producer = Bluefruit callback task, single consumer = loop
static constexpr uint8_t kQueueLen = 8;
static CtrlMsg queue[kQueueLen];
static volatile uint8_t q_head = 0;   // next slot to write (changed only by the callback)
static volatile uint8_t q_tail = 0;   // next slot to read (changed only by loop)

// Notify TX queue. With the default of 3, missing a single event at a 60 ms connection interval fills it
// and notify() fails after waiting 100 ms (seen as notify_fail in B1). ~380 ms worth at 26 packets/s.
static constexpr uint8_t  kHvnQueue = 10;
static constexpr uint16_t kMtuMax = 247;
static constexpr uint16_t kEventLenX125 = 100;   // same as BANDWIDTH_MAX

static char dev_name[20];
static char fw_rev[16];

void sendResp(uint8_t req_id, uint8_t cmd, uint8_t status, const void* payload, uint8_t len) {
    uint8_t buf[icd::kRespMaxLen];
    if (len > sizeof(buf) - icd::kRespHeader) len = sizeof(buf) - icd::kRespHeader;
    buf[0] = req_id;
    buf[1] = cmd;
    buf[2] = status;
    if (len) memcpy(buf + icd::kRespHeader, payload, len);
    const uint16_t h = conn_hdl;
    if (h != BLE_CONN_HANDLE_INVALID && chr_resp->notifyEnabled(h))
        chr_resp->notify(h, buf, icd::kRespHeader + len);
}

static void onCtrlWrite(uint16_t hdl, BLECharacteristic*, uint8_t* data, uint16_t len) {
    (void)hdl;
    if (len < 2) return;   // without req_id there is nothing to reply to
    const uint8_t cmd = data[0], req = data[1];

    if (cmd == icd::CMD_PING) {
        const uint32_t t = timebase_us();   // timestamp as early as possible
        if (len != 2 + sizeof(icd::PingArgs)) {
            sendResp(req, cmd, icd::RS_BAD_LEN);
            return;
        }
        icd::PingPayload p;
        memcpy(&p.nonce, data + 2, sizeof(p.nonce));
        p.t_us = t;
        sendResp(req, cmd, icd::RS_OK, &p, sizeof(p));
        return;
    }

    const uint8_t next = (uint8_t)((q_head + 1) % kQueueLen);
    if (next == q_tail) {
        sendResp(req, cmd, icd::RS_BUSY);
        return;
    }
    CtrlMsg& m = queue[q_head];
    m.len = (uint8_t)(len > icd::kCtrlMaxLen ? icd::kCtrlMaxLen : len);
    memcpy(m.data, data, m.len);
    __DMB();   // move head only after the contents are fully written
    q_head = next;
}

bool popCtrl(CtrlMsg* out) {
    if (q_tail == q_head) return false;
    __DMB();
    *out = queue[q_tail];
    __DMB();
    q_tail = (uint8_t)((q_tail + 1) % kQueueLen);
    return true;
}

// Connection interval 15–30 ms (§4). Windows ignores the PPCP alone, so request it after connecting.
static constexpr uint16_t kConnMinX125 = 12, kConnMaxX125 = 24;
static constexpr uint16_t kSupTimeoutX10ms = 400;   // 4 s

static void onConnect(uint16_t hdl) {
    conn_hdl = hdl;
    BLEConnection* c = Bluefruit.Connection(hdl);
    if (c) {
        c->requestDataLengthUpdate();
        c->requestMtuExchange(kMtuMax);
        const ble_gap_conn_params_t p = {kConnMinX125, kConnMaxX125, 0, kSupTimeoutX10ms};
        sd_ble_gap_conn_param_update(hdl, &p);
    }
}

static void onDisconnect(uint16_t hdl, uint8_t reason) {
    (void)reason;
    if (conn_hdl == hdl) conn_hdl = BLE_CONN_HANDLE_INVALID;
}

static void setupChr(BLECharacteristic*& chr, const uint8_t uuid[16], uint8_t props, uint16_t max_len,
                     bool writable) {
    chr = new BLECharacteristic(BLEUuid(uuid));
    chr->setProperties(props);
    chr->setPermission(SECMODE_OPEN, writable ? SECMODE_OPEN : SECMODE_NO_ACCESS);
    chr->setMaxLen(max_len);
    chr->begin();
}

void begin(uint64_t device_id) {
    // Name: TremorWatch-XXXX (low 16 bits of the device ID)
    snprintf(dev_name, sizeof(dev_name), "TremorWatch-%04X", (unsigned)(device_id & 0xFFFF));
    snprintf(fw_rev, sizeof(fw_rev), "%u.%u.%u", kFwVersion[0], kFwVersion[1], kFwVersion[2]);

    Bluefruit.configPrphConn(kMtuMax, kEventLenX125, kHvnQueue, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    Bluefruit.begin();
    Bluefruit.autoConnLed(false);                   // LEDs are driven by main (S6)
    Bluefruit.setTxPower(4);
    Bluefruit.setName(dev_name);
    Bluefruit.Periph.setConnInterval(12, 24);       // request 15–30 ms (§4)
    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    bledfu.begin();
    bledis.setManufacturer("HYU BPNE");
    bledis.setModel("TremorWatch");
    bledis.setFirmwareRev(fw_rev);
    bledis.begin();
    blebas.begin();
    blebas.write(0);   // real value in S6

    icd::makeUuid128(icd::kUuidService, uuid_svc);
    icd::makeUuid128(icd::kUuidData, uuid_data);
    icd::makeUuid128(icd::kUuidCtrl, uuid_ctrl);
    icd::makeUuid128(icd::kUuidResp, uuid_resp);
    icd::makeUuid128(icd::kUuidStatus, uuid_status);

    svc = new BLEService(BLEUuid(uuid_svc));
    svc->begin();
    setupChr(chr_data, uuid_data, CHR_PROPS_NOTIFY, icd::kDataMaxLen, false);
    setupChr(chr_ctrl, uuid_ctrl, CHR_PROPS_WRITE, icd::kCtrlMaxLen, true);
    chr_ctrl->setWriteCallback(onCtrlWrite);
    setupChr(chr_resp, uuid_resp, CHR_PROPS_NOTIFY, icd::kRespMaxLen, false);
    setupChr(chr_status, uuid_status, CHR_PROPS_NOTIFY | CHR_PROPS_READ, icd::kStatusMaxLen, false);

    // Advertising: flags + Tremor service UUID. The name is long, so it goes in the scan response.
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(*svc);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);     // 20 ms → 152.5 ms
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

bool connected() { return conn_hdl != BLE_CONN_HANDLE_INVALID; }

bool dataSubscribed() {
    const uint16_t h = conn_hdl;
    return h != BLE_CONN_HANDLE_INVALID && chr_data->notifyEnabled(h);
}

uint16_t mtu() {
    const uint16_t h = conn_hdl;
    if (h == BLE_CONN_HANDLE_INVALID) return 0;
    BLEConnection* c = Bluefruit.Connection(h);
    if (!c) return 0;
    // When we request the MTU exchange, Bluefruit stores the peer's server_rx_mtu (Windows: 527) as-is.
    // The effective MTU is the smaller of the two.
    const uint16_t m = c->getMtu();
    return m < kMtuMax ? m : kMtuMax;
}

uint16_t connIntervalX125() {
    const uint16_t h = conn_hdl;
    if (h == BLE_CONN_HANDLE_INVALID) return 0;
    BLEConnection* c = Bluefruit.Connection(h);
    return c ? c->getConnectionInterval() : 0;
}

uint16_t notifyFail() { return notify_fail; }

bool sendData(const icd::DataPkt& pkt) {
    const uint16_t h = conn_hdl;
    if (h == BLE_CONN_HANDLE_INVALID || !chr_data->notifyEnabled(h)) return false;
    if (chr_data->notify(h, &pkt, sizeof(pkt))) return true;
    ++notify_fail;
    return false;
}

void sendStatus(const icd::StatusPkt& st) {
    chr_status->write(&st, sizeof(st));   // Read value
    const uint16_t h = conn_hdl;
    if (h != BLE_CONN_HANDLE_INVALID && chr_status->notifyEnabled(h)) chr_status->notify(h, &st, sizeof(st));
}

}  // namespace ble
