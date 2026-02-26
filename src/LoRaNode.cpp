// ================================================================
//  NODE FIRMWARE v2.2
//  ESP32 DevKit + LoRa SX1278 + Relay
//
//  *** CHANGE NODE_ID BEFORE FLASHING — 1, 2, 3, or 4 ***
//
//  KEY FIXES in this version:
//  - LoRa.endPacket(true) — synchronous TX, so we are back in RX
//    mode before the gateway's next command arrives
//  - POST_TX_SETTLE_MS delay after endPacket before receive()
//  - RELAY_PIN = 13 (corrected per hardware)
//  - No RTC, No moisture sensor
//  - Scheduling fully owned by gateway RTC
//  - Auto valve-off via millis() — works even if gateway goes down
// ================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ================================================================
//  *** CHANGE THIS PER NODE ***
// ================================================================
#define NODE_ID  1      // Valid: 1, 2, 3, 4

static_assert(NODE_ID >= 1 && NODE_ID <= 4, "NODE_ID must be 1-4");

// ================================================================
//  LORA PINS  (ESP32 DevKit, VSPI)
// ================================================================
#define LORA_CS    4
#define LORA_SCK   5
#define LORA_MOSI  6
#define LORA_MISO  7
#define LORA_RST   15
#define LORA_DIO0  16
#define LORA_FREQ  433E6

// ================================================================
//  RELAY PIN
// ================================================================
#define RELAY_PIN  13    // HIGH = valve open, LOW = valve closed

// ================================================================
//  PROTOCOL — must match gateway exactly
// ================================================================
#define PKT_LEN    13
#define PKT_START  0xAA
#define PKT_END    0x55

#define CMD_VALVE_ON      0x01
#define CMD_VALVE_OFF     0x02
#define CMD_SET_SCHEDULE  0x03   // ACK only — no local RTC
#define CMD_CLR_SCHEDULE  0x04   // ACK only — no local RTC
#define CMD_STATUS_REQ    0x05
#define CMD_ACK           0x10
#define CMD_HEARTBEAT     0x11

#define IDX_START  0
#define IDX_DST    1
#define IDX_SRC    2
#define IDX_TXID   3
#define IDX_CMD    4
#define IDX_P0     5
#define IDX_P1     6
#define IDX_P2     7
#define IDX_P3     8
#define IDX_P4     9
#define IDX_P5     10
#define IDX_CRC    11
#define IDX_END    12

#define GATEWAY_ID  0

// ================================================================
//  TIMING
// ================================================================
#define HEARTBEAT_INTERVAL_MS  60000UL
#define POST_TX_SETTLE_MS         20UL   // settle after TX before RX

// ================================================================
//  GLOBALS
// ================================================================
bool          valveOpen    = false;
unsigned long valveOnAt    = 0;
uint16_t      valveRuntime = 0;     // seconds; 0 = no auto-off

unsigned long lastHeartbeat  = 0;
uint8_t       lastRxTxID     = 0;
bool          lastTxIDValid  = false;

// ================================================================
//  CRC8
// ================================================================
static uint8_t crc8(const uint8_t* d, uint8_t len) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= d[i];
    return c;
}

// ================================================================
//  VALVE
// ================================================================
static void openValve(uint16_t durationSec) {
    digitalWrite(RELAY_PIN, HIGH);
    valveOpen    = true;
    valveOnAt    = millis();
    valveRuntime = durationSec;
    Serial.printf("[VALVE] OPEN %us\n", durationSec);
}

static void closeValve() {
    digitalWrite(RELAY_PIN, LOW);
    valveOpen    = false;
    valveRuntime = 0;
    Serial.println("[VALVE] CLOSED");
}

// ================================================================
//  SEND ACK
//  endPacket(true) = synchronous TX — waits until packet fully sent
//  before returning. Gateway's waitForAck window is open, and our
//  RX is resumed after settle delay.
// ================================================================
static void sendAck(uint8_t txID) {
    uint8_t pkt[PKT_LEN];
    pkt[IDX_START] = PKT_START;
    pkt[IDX_DST]   = GATEWAY_ID;
    pkt[IDX_SRC]   = NODE_ID;
    pkt[IDX_TXID]  = txID;
    pkt[IDX_CMD]   = CMD_ACK;
    pkt[IDX_P0]    = 0;
    pkt[IDX_P1]    = 0;
    pkt[IDX_P2]    = 0;
    pkt[IDX_P3]    = valveOpen ? 1u : 0u;  // valve state
    pkt[IDX_P4]    = 0;                    // no moisture sensor
    pkt[IDX_P5]    = 0;
    pkt[IDX_CRC]   = crc8(pkt, IDX_CRC);
    pkt[IDX_END]   = PKT_END;

    LoRa.beginPacket();
    LoRa.write(pkt, PKT_LEN);
    LoRa.endPacket(true);           // <<< synchronous TX
    delay(POST_TX_SETTLE_MS);
    LoRa.receive();

    Serial.printf("[ACK] txID=0x%02X Valve=%s\n",
        txID, valveOpen ? "ON" : "OFF");
}

// ================================================================
//  SEND HEARTBEAT
// ================================================================
static void sendHeartbeat() {
    uint8_t pkt[PKT_LEN];
    pkt[IDX_START] = PKT_START;
    pkt[IDX_DST]   = GATEWAY_ID;
    pkt[IDX_SRC]   = NODE_ID;
    pkt[IDX_TXID]  = 0xFF;           // reserved for heartbeats
    pkt[IDX_CMD]   = CMD_HEARTBEAT;
    pkt[IDX_P0]    = valveOpen ? 1u : 0u;
    pkt[IDX_P1]    = 0;              // no moisture sensor
    pkt[IDX_P2]    = 0;
    pkt[IDX_P3]    = 0;
    pkt[IDX_P4]    = 0;
    pkt[IDX_P5]    = 0;
    pkt[IDX_CRC]   = crc8(pkt, IDX_CRC);
    pkt[IDX_END]   = PKT_END;

    LoRa.beginPacket();
    LoRa.write(pkt, PKT_LEN);
    LoRa.endPacket(true);           // <<< synchronous TX
    delay(POST_TX_SETTLE_MS);
    LoRa.receive();

    Serial.printf("[HB] Valve:%s\n", valveOpen ? "ON" : "OFF");
}

// ================================================================
//  FORWARD PACKET (relay for another node)
//  Gateway sets IDX_P5 = this node's ID as relay request.
//  We clear relay field, recompute CRC, retransmit.
// ================================================================
static void forwardPacket(uint8_t* buf) {
    buf[IDX_P5] = 0;
    buf[IDX_CRC] = crc8(buf, IDX_CRC);

    LoRa.beginPacket();
    LoRa.write(buf, PKT_LEN);
    LoRa.endPacket(true);           // <<< synchronous TX
    delay(POST_TX_SETTLE_MS);
    LoRa.receive();

    Serial.printf("[RELAY] -> Node%d\n", buf[IDX_DST]);
}

// ================================================================
//  PROCESS COMMAND addressed to this node
// ================================================================
static void processCommand(uint8_t* buf) {
    uint8_t cmd  = buf[IDX_CMD];
    uint8_t txID = buf[IDX_TXID];

    switch (cmd) {

        case CMD_VALVE_ON: {
            uint16_t dur = ((uint16_t)buf[IDX_P0] << 8) | buf[IDX_P1];
            openValve(dur);
            sendAck(txID);
            break;
        }

        case CMD_VALVE_OFF:
            closeValve();
            sendAck(txID);
            break;

        case CMD_STATUS_REQ:
            sendAck(txID);
            break;

        case CMD_SET_SCHEDULE:
        case CMD_CLR_SCHEDULE:
            // Node has no RTC — scheduling runs on gateway only.
            // ACK so gateway does not retry and mark node offline.
            Serial.println("[CMD] Schedule cmd — ACK only (no RTC)");
            sendAck(txID);
            break;

        default:
            Serial.printf("[CMD] Unknown 0x%02X — ACKing\n", cmd);
            sendAck(txID);
            break;
    }
}

// ================================================================
//  LORA RX: read, validate, relay-or-dispatch
// ================================================================
static void receiveLoRa() {
    int sz = LoRa.parsePacket();
    if (!sz) return;

    uint8_t buf[PKT_LEN] = {0};
    int n = (sz < PKT_LEN) ? sz : PKT_LEN;
    for (int i = 0; i < n; i++) buf[i] = LoRa.read();
    while (LoRa.available()) LoRa.read();

    // Framing
    if (buf[IDX_START] != PKT_START) return;
    if (buf[IDX_END]   != PKT_END)   return;

    // CRC
    if (crc8(buf, IDX_CRC) != buf[IDX_CRC]) {
        Serial.println("[RX] CRC fail"); return;
    }

    uint8_t dst   = buf[IDX_DST];
    uint8_t txID  = buf[IDX_TXID];
    uint8_t relay = buf[IDX_P5];

    // Not for us — relay if instructed
    if (dst != NODE_ID) {
        if (relay == NODE_ID) forwardPacket(buf);
        return;
    }

    // Addressed to us — duplicate suppression
    // Same txID = retransmit, re-ACK without re-executing command
    if (lastTxIDValid && txID == lastRxTxID && txID != 0xFF) {
        Serial.printf("[RX] Dup txID=0x%02X re-ACK\n", txID);
        sendAck(txID);
        return;
    }

    lastRxTxID    = txID;
    lastTxIDValid = true;

    Serial.printf("[RX] CMD=0x%02X txID=0x%02X\n", buf[IDX_CMD], txID);
    processCommand(buf);
}

// ================================================================
//  AUTO OFF: millis()-based runtime (no RTC needed)
//  Gateway sends duration with CMD_VALVE_ON.
//  Node enforces it locally — valve closes even if gateway is down.
// ================================================================
static void checkAutoOff() {
    if (!valveOpen || valveRuntime == 0) return;
    if (millis() - valveOnAt >= (unsigned long)valveRuntime * 1000UL) {
        Serial.println("[AUTO-OFF] Runtime done");
        closeValve();
    }
}

// ================================================================
//  HEARTBEAT
// ================================================================
static void checkHeartbeat() {
    if (millis() - lastHeartbeat < HEARTBEAT_INTERVAL_MS) return;
    lastHeartbeat = millis();
    sendHeartbeat();
}

// ================================================================
//  LORA INIT
// ================================================================
static void setupLoRa() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);  delay(20);
    digitalWrite(LORA_RST, HIGH); delay(50);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LoRa] INIT FAILED");
        while (true) delay(1000);
    }
    // Must exactly match gateway settings
    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    LoRa.receive();
    Serial.println("[LoRa] Ready");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.printf("\n[BOOT] Node %d\n", NODE_ID);

    // Relay — force LOW immediately (valve CLOSED on boot)
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    setupLoRa();

    // Fire first heartbeat immediately to register with gateway
    lastHeartbeat = millis() - HEARTBEAT_INTERVAL_MS;
    sendHeartbeat();

    Serial.printf("[BOOT] Node %d ready\n", NODE_ID);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
    receiveLoRa();      // receive + dispatch
    checkAutoOff();     // millis runtime enforcement
    checkHeartbeat();   // 60s keep-alive
}