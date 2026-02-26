// ================================================================
//  GATEWAY FIRMWARE v2.2
//  ESP32-S3 + LoRa + Telegram + DS3231 RTC
//
//  KEY FIXES in this version:
//  - LoRa.endPacket(true) — synchronous TX so RX mode is ready
//    before the node's ACK arrives (was the primary no-ACK cause)
//  - receiveLoRa() no longer consumes CMD_ACK packets, preventing
//    them from being eaten before waitForAck() can read them
//  - ACK timeout raised to 2000ms for safety margin
//  - RTC pins corrected: SDA=8, SCL=18
//  - Small post-TX settle delay before listening for ACK
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>
#include <RTClib.h>
#include <Wire.h>

// ================================================================
//  CREDENTIALS
// ================================================================
static const char*  WIFI_SSID = "Phoenix";
static const char*  WIFI_PASS = "Harsha dj";
static const char*  BOT_TOKEN = "8248041417:AAEfOcW1aVOJ0Z6PEa81Yjia0MdECBpgQ7s";
static const String CHAT_ID   = "885968591";

// ================================================================
//  LORA PINS  (ESP32-S3, HSPI bus)
// ================================================================
#define LORA_CS    4
#define LORA_SCK   5
#define LORA_MOSI  6
#define LORA_MISO  7
#define LORA_RST   15
#define LORA_DIO0  16
#define LORA_FREQ  433E6

// ================================================================
//  RTC PINS  (DS3231, I2C)
// ================================================================
#define RTC_SDA  8
#define RTC_SCL  18

// ================================================================
//  PROTOCOL
// ================================================================
#define PKT_LEN    13
#define PKT_START  0xAA
#define PKT_END    0x55

// Commands (gateway -> node)
#define CMD_VALVE_ON       0x01
#define CMD_VALVE_OFF      0x02
#define CMD_SET_SCHEDULE   0x03
#define CMD_CLR_SCHEDULE   0x04
#define CMD_STATUS_REQ     0x05
// Responses (node -> gateway)
#define CMD_ACK            0x10
#define CMD_HEARTBEAT      0x11
#define CMD_ALERT          0x12

// Packet byte indices
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

#define GATEWAY_ID    0
#define NUM_NODES     4
#define MAX_SCHEDULES 3

// ================================================================
//  TIMING
// ================================================================
// Raised to 2000ms — at SF9/BW125 each packet is ~260ms air time,
// so TX + node processing + ACK TX needs comfortable headroom
#define ACK_TIMEOUT_MS        2000UL
#define POST_TX_SETTLE_MS       20UL   // wait after TX before listening
#define HEARTBEAT_TIMEOUT_MS 180000UL
#define HB_CHECK_INTERVAL_MS  30000UL
#define TELEGRAM_POLL_MS       2000UL
#define WIFI_RETRY_MS         10000UL
#define MAX_TX_RETRIES             3

// ================================================================
//  STRUCTS
// ================================================================
struct Schedule {
    bool     active;
    uint8_t  hour;
    uint8_t  minute;
    uint16_t duration;  // seconds
    uint8_t  days;      // bitmask: bit0=Sun..bit6=Sat, 0x7F=daily

    Schedule()
        : active(false), hour(6), minute(0), duration(300), days(0x7F) {}
    Schedule(bool a, uint8_t h, uint8_t m, uint16_t d, uint8_t w)
        : active(a), hour(h), minute(m), duration(d), days(w) {}
};

struct NodeState {
    bool          valveOpen;
    bool          online;
    uint8_t       moisture;
    unsigned long lastSeen;
    unsigned long valveOnAt;
    uint16_t      runtime;
    Schedule      schedules[MAX_SCHEDULES];
    bool          alertSent;

    NodeState()
        : valveOpen(false), online(false), moisture(0),
          lastSeen(0), valveOnAt(0), runtime(0), alertSent(false) {}
};

// ================================================================
//  GLOBALS
// ================================================================
SPIClass             spiLoRa(HSPI);
WiFiClientSecure     secClient;
UniversalTelegramBot bot(BOT_TOKEN, secClient);
Preferences          prefs;
RTC_DS3231           rtc;
bool                 rtcOK = false;

NodeState     nodes[NUM_NODES + 1];  // index 1-4, 0 unused
uint8_t       txID = 0;

unsigned long lastTelegramPoll = 0;
unsigned long lastHBCheck      = 0;
unsigned long lastWiFiRetry    = 0;
uint8_t       lastSchedMinute  = 255;

// ================================================================
//  CRC8
// ================================================================
static uint8_t crc8(const uint8_t* d, uint8_t len) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= d[i];
    return c;
}

// ================================================================
//  TELEGRAM: safe send with one retry
// ================================================================
static void tgSend(const String& msg) {
    for (int i = 0; i < 2; i++) {
        if (bot.sendMessage(CHAT_ID, msg, "")) return;
        delay(500);
    }
    Serial.println("[TG] send failed");
}

// ================================================================
//  WIFI
// ================================================================
static void wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("[WiFi] Connecting");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000UL) {
        delay(500); Serial.print('.');
    }
    Serial.println(WiFi.status() == WL_CONNECTED
        ? "\n[WiFi] Connected"
        : "\n[WiFi] FAILED — will retry");
}

static void checkWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    if (millis() - lastWiFiRetry < WIFI_RETRY_MS) return;
    lastWiFiRetry = millis();
    WiFi.disconnect();
    delay(100);
    wifiConnect();
}

// ================================================================
//  LORA INIT
// ================================================================
static void setupLoRa() {
    spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(spiLoRa);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    // Hardware reset
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);  delay(20);
    digitalWrite(LORA_RST, HIGH); delay(50);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LoRa] INIT FAILED");
        while (true) delay(1000);
    }
    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    LoRa.receive();
    Serial.println("[LoRa] Ready");
}

// ================================================================
//  SEND RAW PACKET
//  Uses endPacket(true) = SYNCHRONOUS — TX fully complete before
//  returning, so LoRa.receive() is active before node ACK arrives.
// ================================================================
static void sendRawPacket(uint8_t dst, uint8_t id, uint8_t cmd,
                          uint8_t p0, uint8_t p1, uint8_t p2,
                          uint8_t p3, uint8_t p4, uint8_t p5) {
    uint8_t pkt[PKT_LEN];
    pkt[IDX_START] = PKT_START;
    pkt[IDX_DST]   = dst;
    pkt[IDX_SRC]   = GATEWAY_ID;
    pkt[IDX_TXID]  = id;
    pkt[IDX_CMD]   = cmd;
    pkt[IDX_P0]    = p0;
    pkt[IDX_P1]    = p1;
    pkt[IDX_P2]    = p2;
    pkt[IDX_P3]    = p3;
    pkt[IDX_P4]    = p4;
    pkt[IDX_P5]    = p5;
    pkt[IDX_CRC]   = crc8(pkt, IDX_CRC);
    pkt[IDX_END]   = PKT_END;

    LoRa.beginPacket();
    LoRa.write(pkt, PKT_LEN);
    LoRa.endPacket(true);       // <<< TRUE = synchronous, waits for TX done
    delay(POST_TX_SETTLE_MS);   // brief settle before switching to RX
    LoRa.receive();
}

// ================================================================
//  WAIT FOR ACK
//  IMPORTANT: receiveLoRa() in the main loop does NOT consume
//  CMD_ACK packets, so they are always available here.
// ================================================================
static bool waitForAck(uint8_t target, uint8_t expectedID) {
    unsigned long start = millis();

    while (millis() - start < ACK_TIMEOUT_MS) {
        int sz = LoRa.parsePacket();
        if (!sz) continue;

        uint8_t buf[PKT_LEN] = {0};
        int n = (sz < PKT_LEN) ? sz : PKT_LEN;
        for (int i = 0; i < n; i++) buf[i] = LoRa.read();
        while (LoRa.available()) LoRa.read();

        // Framing
        if (buf[IDX_START] != PKT_START) continue;
        if (buf[IDX_END]   != PKT_END)   continue;

        // CRC
        if (crc8(buf, IDX_CRC) != buf[IDX_CRC]) {
            Serial.println("[ACK] CRC fail"); continue;
        }

        // We only want: CMD_ACK, to us, from the right node, matching txID
        if (buf[IDX_CMD]  != CMD_ACK)      continue;
        if (buf[IDX_DST]  != GATEWAY_ID)   continue;
        if (buf[IDX_SRC]  != target)       continue;
        if (buf[IDX_TXID] != expectedID)   continue;

        // Update node state
        nodes[target].valveOpen = (buf[IDX_P3] != 0);
        nodes[target].moisture  = buf[IDX_P4];
        nodes[target].online    = true;
        nodes[target].lastSeen  = millis();
        nodes[target].alertSent = false;

        Serial.printf("[ACK] Node%d Valve:%s\n",
            target, nodes[target].valveOpen ? "ON" : "OFF");
        return true;
    }

    Serial.printf("[ACK] TIMEOUT node%d\n", target);
    return false;
}

// ================================================================
//  SEND COMMAND: direct attempts then relay fallback
// ================================================================
static bool sendCommand(uint8_t target, uint8_t cmd,
                        uint8_t p0 = 0, uint8_t p1 = 0, uint8_t p2 = 0,
                        uint8_t p3 = 0, uint8_t p4 = 0, uint8_t p5 = 0) {
    txID++;
    if (txID == 0xFF) txID = 0;  // 0xFF reserved for heartbeats

    // Direct attempts
    for (int i = 0; i < MAX_TX_RETRIES; i++) {
        Serial.printf("[TX] Attempt %d -> Node%d cmd=0x%02X\n", i+1, target, cmd);
        sendRawPacket(target, txID, cmd, p0, p1, p2, p3, p4, p5);
        if (waitForAck(target, txID)) return true;
        delay(100);
    }

    // Relay fallback: use any online node as bridge
    Serial.println("[TX] Direct failed, trying relay...");
    for (int relay = 1; relay <= NUM_NODES; relay++) {
        if (relay == (int)target)    continue;
        if (!nodes[relay].online)   continue;
        Serial.printf("[TX] Relay via Node%d\n", relay);
        // p5 = relay node ID; relay node sees dst!=self, p5==self, forwards
        sendRawPacket(target, txID, cmd, p0, p1, p2, p3, p4, (uint8_t)relay);
        if (waitForAck(target, txID)) return true;
    }

    nodes[target].online = false;
    Serial.printf("[TX] Node%d unreachable\n", target);
    return false;
}

// ================================================================
//  NVS PERSISTENCE
// ================================================================
static void saveSchedule(uint8_t valve, uint8_t slot) {
    if (valve < 1 || valve > NUM_NODES || slot >= MAX_SCHEDULES) return;
    prefs.begin("gw_sc", false);
    char k[12];
    const Schedule& s = nodes[valve].schedules[slot];
    snprintf(k, sizeof(k), "v%us%ua", valve, slot); prefs.putUChar(k,  s.active ? 1u : 0u);
    snprintf(k, sizeof(k), "v%us%uh", valve, slot); prefs.putUChar(k,  s.hour);
    snprintf(k, sizeof(k), "v%us%um", valve, slot); prefs.putUChar(k,  s.minute);
    snprintf(k, sizeof(k), "v%us%ud", valve, slot); prefs.putUShort(k, s.duration);
    snprintf(k, sizeof(k), "v%us%uw", valve, slot); prefs.putUChar(k,  s.days);
    prefs.end();
}

static void loadAllSchedules() {
    prefs.begin("gw_sc", true);
    char k[12];
    for (uint8_t v = 1; v <= NUM_NODES; v++) {
        for (uint8_t s = 0; s < MAX_SCHEDULES; s++) {
            Schedule& sc = nodes[v].schedules[s];
            snprintf(k, sizeof(k), "v%us%ua", v, s); sc.active   = prefs.getUChar(k, 0) != 0;
            snprintf(k, sizeof(k), "v%us%uh", v, s); sc.hour     = prefs.getUChar(k, 6);
            snprintf(k, sizeof(k), "v%us%um", v, s); sc.minute   = prefs.getUChar(k, 0);
            snprintf(k, sizeof(k), "v%us%ud", v, s); sc.duration = prefs.getUShort(k, 300);
            snprintf(k, sizeof(k), "v%us%uw", v, s); sc.days     = prefs.getUChar(k, 0x7F);
        }
    }
    prefs.end();
    Serial.println("[NVS] Schedules loaded");
}

// ================================================================
//  RTC SCHEDULE CHECK (fires once per minute)
// ================================================================
static void checkSchedules() {
    if (!rtcOK) return;
    DateTime now = rtc.now();
    uint8_t curMin = now.minute();
    if (curMin == lastSchedMinute) return;
    lastSchedMinute = curMin;

    uint8_t dayBit = (uint8_t)(1u << now.dayOfTheWeek());

    for (uint8_t v = 1; v <= NUM_NODES; v++) {
        for (uint8_t s = 0; s < MAX_SCHEDULES; s++) {
            const Schedule& sc = nodes[v].schedules[s];
            if (!sc.active)               continue;
            if (!(sc.days & dayBit))      continue;
            if (sc.hour   != now.hour())  continue;
            if (sc.minute != now.minute()) continue;

            if (nodes[v].valveOpen) {
                Serial.printf("[SCHED] V%d already open, skip\n", v);
                continue;
            }

            Serial.printf("[SCHED] Fire V%d slot%d\n", v, s);
            bool ok = sendCommand(v, CMD_VALVE_ON,
                                  (uint8_t)(sc.duration >> 8),
                                  (uint8_t)(sc.duration & 0xFF));
            if (ok) {
                nodes[v].valveOnAt = millis();
                nodes[v].runtime   = sc.duration;
                tgSend("⏰ Schedule: Valve " + String(v) +
                       " ON for " + String(sc.duration) + "s");
            } else {
                tgSend("⚠️ Schedule FAILED: Valve " + String(v));
            }
        }
    }
}

// ================================================================
//  AUTO OFF
// ================================================================
static void checkAutoOff() {
    for (uint8_t v = 1; v <= NUM_NODES; v++) {
        if (!nodes[v].valveOpen || nodes[v].runtime == 0) continue;
        if (millis() - nodes[v].valveOnAt < (unsigned long)nodes[v].runtime * 1000UL) continue;

        Serial.printf("[AUTO-OFF] V%d\n", v);
        bool ok = sendCommand(v, CMD_VALVE_OFF);
        if (ok) {
            nodes[v].valveOpen = false;
            nodes[v].runtime   = 0;
            tgSend("✅ Valve " + String(v) + " auto-OFF");
        } else {
            // Reset timer — retry at next loop iteration after runtime again
            nodes[v].valveOnAt = millis();
            tgSend("⚠️ Valve " + String(v) + " auto-OFF failed — retrying");
        }
    }
}

// ================================================================
//  RECEIVE UNSOLICITED LoRa (heartbeats + alerts ONLY)
//
//  CRITICAL: This function deliberately ignores CMD_ACK packets.
//  If it consumed ACKs, waitForAck() would time out because the
//  ACK would already be drained from the LoRa FIFO here.
// ================================================================
static void receiveLoRa() {
    int sz = LoRa.parsePacket();
    if (!sz) return;

    uint8_t buf[PKT_LEN] = {0};
    int n = (sz < PKT_LEN) ? sz : PKT_LEN;
    for (int i = 0; i < n; i++) buf[i] = LoRa.read();
    while (LoRa.available()) LoRa.read();

    if (buf[IDX_START] != PKT_START) return;
    if (buf[IDX_END]   != PKT_END)   return;
    if (crc8(buf, IDX_CRC) != buf[IDX_CRC]) {
        Serial.println("[RX] CRC fail"); return;
    }

    uint8_t cmd = buf[IDX_CMD];

    // ACK packets belong to waitForAck() — do NOT consume them here
    if (cmd == CMD_ACK) return;

    uint8_t src = buf[IDX_SRC];
    if (src < 1 || src > NUM_NODES) return;

    if (cmd == CMD_HEARTBEAT) {
        nodes[src].valveOpen = (buf[IDX_P0] != 0);
        nodes[src].moisture  = buf[IDX_P1];
        nodes[src].online    = true;
        nodes[src].lastSeen  = millis();
        nodes[src].alertSent = false;
        Serial.printf("[HB] Node%d Valve:%s\n",
            src, buf[IDX_P0] ? "ON" : "OFF");
        return;
    }

    if (cmd == CMD_ALERT) {
        String msg = "🚨 Valve " + String(src) + ": ";
        switch (buf[IDX_P0]) {
            case 0x01: msg += "Low pressure";   break;
            case 0x02: msg += "Flow fault";     break;
            case 0x03: msg += "Moisture low";   break;
            case 0x04: msg += "Relay stuck";    break;
            default:   msg += "Error 0x" + String(buf[IDX_P0], HEX);
        }
        tgSend(msg);
    }
}

// ================================================================
//  HEARTBEAT CHECK
// ================================================================
static void checkHeartbeat() {
    if (millis() - lastHBCheck < HB_CHECK_INTERVAL_MS) return;
    lastHBCheck = millis();
    for (uint8_t v = 1; v <= NUM_NODES; v++) {
        if (nodes[v].lastSeen == 0) continue;
        bool timedOut = (millis() - nodes[v].lastSeen > HEARTBEAT_TIMEOUT_MS);
        if (timedOut && nodes[v].online && !nodes[v].alertSent) {
            nodes[v].online    = false;
            nodes[v].alertSent = true;
            tgSend("⚠️ Valve " + String(v) + " OFFLINE");
        }
    }
}

// ================================================================
//  TELEGRAM HELPERS
// ================================================================
static String timeStr(uint8_t h, uint8_t m) {
    char buf[6]; snprintf(buf, sizeof(buf), "%02u:%02u", h, m);
    return String(buf);
}

static void sendMainMenu() {
    String kb =
        "[[\"💧 V1 ON\",\"⛔ V1 OFF\",\"📊 V1 Info\"],"
        "[\"💧 V2 ON\",\"⛔ V2 OFF\",\"📊 V2 Info\"],"
        "[\"💧 V3 ON\",\"⛔ V3 OFF\",\"📊 V3 Info\"],"
        "[\"💧 V4 ON\",\"⛔ V4 OFF\",\"📊 V4 Info\"],"
        "[\"📅 Schedules\",\"📡 Status\",\"🕐 RTC\"]]";
    bot.sendMessageWithReplyKeyboard(CHAT_ID, "🌱 Irrigation Gateway", "", kb, true);
}

static void sendSystemStatus() {
    String msg = "📡 System Status\n";
    if (rtcOK) {
        DateTime now = rtc.now();
        char tb[20]; snprintf(tb, sizeof(tb), "🕐 %02d:%02d:%02d\n\n",
            now.hour(), now.minute(), now.second());
        msg += tb;
    }
    for (uint8_t v = 1; v <= NUM_NODES; v++) {
        msg += "Valve " + String(v) + ": ";
        msg += nodes[v].valveOpen ? "💧 ON  " : "⭕ OFF ";
        msg += nodes[v].online    ? "✅\n"     : "❌ Offline\n";
    }
    tgSend(msg);
}

static void sendValveInfo(uint8_t v) {
    sendCommand(v, CMD_STATUS_REQ);  // refresh state
    String msg = "📊 Valve " + String(v) + "\n";
    msg += nodes[v].valveOpen ? "State : 💧 ON\n" : "State : ⭕ OFF\n";
    msg += nodes[v].online    ? "Link  : ✅\n"     : "Link  : ❌ Offline\n";
    msg += "\nSchedules:\n";
    for (uint8_t s = 0; s < MAX_SCHEDULES; s++) {
        const Schedule& sc = nodes[v].schedules[s];
        msg += "  Slot " + String(s+1) + ": ";
        msg += sc.active
            ? timeStr(sc.hour, sc.minute) + " | " + String(sc.duration) + "s\n"
            : "Not set\n";
    }
    tgSend(msg);
}

static void sendScheduleHelp() {
    tgSend(
        "📅 Schedule Commands\n\n"
        "Set:   SET V1 SCHED 1 06:30 600\n"
        "       (valve 1-4, slot 1-3, HH:MM, duration sec)\n\n"
        "Clear: CLR V1 SCHED 1\n\n"
        "Time:  SET TIME 14:30"
    );
}

static void sendRTCStatus() {
    if (!rtcOK) { tgSend("❌ RTC not found"); return; }
    DateTime now = rtc.now();
    char buf[64];
    snprintf(buf, sizeof(buf), "🕐 %04d-%02d-%02d %02d:%02d:%02d\nTemp: %.1f°C",
        now.year(), now.month(), now.day(),
        now.hour(), now.minute(), now.second(),
        rtc.getTemperature());
    tgSend(String(buf));
}

// ================================================================
//  COMMAND PARSERS
// ================================================================
static uint8_t parseDayMask(const String& s) {
    if (s.length() < 7) return 0x7F;
    uint8_t m = 0;
    if (s[0] != '-') m |= 0x01;
    if (s[1] != '-') m |= 0x02;
    if (s[2] != '-') m |= 0x04;
    if (s[3] != '-') m |= 0x08;
    if (s[4] != '-') m |= 0x10;
    if (s[5] != '-') m |= 0x20;
    if (s[6] != '-') m |= 0x40;
    return m;
}

static void handleSetSchedule(const String& text) {
    // Format: SET V1 SCHED 1 06:30 600 [SMTWTFS]
    if (text.length() < 22) { tgSend("❌ Format: SET V1 SCHED 1 06:30 600"); return; }

    int valve = text.charAt(5) - '0';
    if (valve < 1 || valve > NUM_NODES) { tgSend("❌ Valve 1-4"); return; }

    int sp = text.indexOf("SCHED ");
    if (sp < 0) { tgSend("❌ Missing SCHED"); return; }

    int slot = text.charAt(sp + 6) - '1';
    if (slot < 0 || slot >= MAX_SCHEDULES) { tgSend("❌ Slot 1-3"); return; }

    int tp = sp + 8;
    if (tp + 5 >= (int)text.length()) { tgSend("❌ Missing HH:MM"); return; }

    int h = text.substring(tp,     tp+2).toInt();
    int m = text.substring(tp+3,   tp+5).toInt();
    if (h < 0 || h > 23 || m < 0 || m > 59) { tgSend("❌ Bad time"); return; }

    int dp = tp + 6;
    if (dp >= (int)text.length()) { tgSend("❌ Missing duration"); return; }

    int de = text.indexOf(' ', dp);
    int dur;
    uint8_t mask = 0x7F;
    if (de < 0) {
        dur = text.substring(dp).toInt();
    } else {
        dur = text.substring(dp, de).toInt();
        String ds = text.substring(de + 1); ds.trim();
        mask = parseDayMask(ds);
    }
    if (dur < 1 || dur > 7200) { tgSend("❌ Duration 1-7200s"); return; }

    Schedule& sc = nodes[valve].schedules[slot];
    sc.active   = true;
    sc.hour     = (uint8_t)h;
    sc.minute   = (uint8_t)m;
    sc.duration = (uint16_t)dur;
    sc.days     = mask;
    saveSchedule((uint8_t)valve, (uint8_t)slot);

    bool ok = sendCommand((uint8_t)valve, CMD_SET_SCHEDULE,
                          (uint8_t)slot, (uint8_t)h, (uint8_t)m,
                          (uint8_t)(dur>>8), (uint8_t)(dur&0xFF), mask);
    tgSend(ok
        ? "✅ Sched set V" + String(valve) + " slot" + String(slot+1) +
          " @ " + timeStr(h,m) + " " + String(dur) + "s"
        : "⚠️ Saved locally — node unreachable");
}

static void handleClrSchedule(const String& text) {
    if (text.length() < 14) { tgSend("❌ Format: CLR V1 SCHED 1"); return; }
    int valve = text.charAt(5) - '0';
    if (valve < 1 || valve > NUM_NODES) { tgSend("❌ Valve 1-4"); return; }
    int sp = text.indexOf("SCHED ");
    if (sp < 0) { tgSend("❌ Missing SCHED"); return; }
    int slot = text.charAt(sp + 6) - '1';
    if (slot < 0 || slot >= MAX_SCHEDULES) { tgSend("❌ Slot 1-3"); return; }

    nodes[valve].schedules[slot].active = false;
    saveSchedule((uint8_t)valve, (uint8_t)slot);
    bool ok = sendCommand((uint8_t)valve, CMD_CLR_SCHEDULE, (uint8_t)slot);
    tgSend(ok ? "🗑 Cleared V" + String(valve) + " slot" + String(slot+1)
              : "⚠️ Cleared locally — node unreachable");
}

static void handleSetTime(const String& text) {
    if (text.length() < 13) { tgSend("❌ Format: SET TIME HH:MM"); return; }
    int h = text.substring(9,  11).toInt();
    int m = text.substring(12, 14).toInt();
    if (h < 0 || h > 23 || m < 0 || m > 59) { tgSend("❌ Bad time"); return; }
    if (!rtcOK) { tgSend("❌ RTC not available"); return; }
    DateTime now = rtc.now();
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), (uint8_t)h, (uint8_t)m, 0));
    tgSend("🕐 RTC set to " + timeStr(h, m));
}

// ================================================================
//  TELEGRAM POLL + DISPATCH
// ================================================================
static void handleTelegram() {
    if (millis() - lastTelegramPoll < TELEGRAM_POLL_MS) return;
    if (WiFi.status() != WL_CONNECTED) return;
    lastTelegramPoll = millis();

    int n = bot.getUpdates(bot.last_message_received + 1);
    while (n > 0) {
        for (int i = 0; i < n; i++) {
            String text = bot.messages[i].text;
            text.trim();
            Serial.println("[TG] " + text);

            if (text == "/start" || text == "Menu")     { sendMainMenu();    continue; }
            if (text == "📡 Status")                    { sendSystemStatus(); continue; }
            if (text == "📅 Schedules")                 { sendScheduleHelp(); continue; }
            if (text == "🕐 RTC")                       { sendRTCStatus();   continue; }

            bool handled = false;
            for (uint8_t v = 1; v <= NUM_NODES && !handled; v++) {
                String vn = String(v);

                if (text == "💧 V" + vn + " ON") {
                    uint16_t rt = nodes[v].schedules[0].active
                                ? nodes[v].schedules[0].duration : 300u;
                    bool ok = sendCommand(v, CMD_VALVE_ON,
                                         (uint8_t)(rt>>8), (uint8_t)(rt&0xFF));
                    if (ok) { nodes[v].valveOnAt = millis(); nodes[v].runtime = rt; }
                    tgSend(ok ? "💧 Valve " + vn + " ON | " + String(rt) + "s"
                              : "❌ Valve " + vn + " FAILED — no ACK");
                    handled = true;
                }
                else if (text == "⛔ V" + vn + " OFF") {
                    bool ok = sendCommand(v, CMD_VALVE_OFF);
                    if (ok) { nodes[v].valveOpen = false; nodes[v].runtime = 0; }
                    tgSend(ok ? "⛔ Valve " + vn + " OFF"
                              : "❌ Valve " + vn + " OFF FAILED");
                    handled = true;
                }
                else if (text == "📊 V" + vn + " Info") {
                    sendValveInfo(v); handled = true;
                }
            }

            if (!handled) {
                if      (text.startsWith("SET V") && text.indexOf("SCHED") > 0) handleSetSchedule(text);
                else if (text.startsWith("CLR V") && text.indexOf("SCHED") > 0) handleClrSchedule(text);
                else if (text.startsWith("SET TIME "))                           handleSetTime(text);
                else tgSend("❓ Unknown — send /start for menu");
            }
        }
        n = bot.getUpdates(bot.last_message_received + 1);
    }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[BOOT] Gateway v2.2");

    // RTC on custom I2C pins
    Wire.begin(RTC_SDA, RTC_SCL);
    if (rtc.begin(&Wire)) {
        rtcOK = true;
        if (rtc.lostPower()) {
            Serial.println("[RTC] Lost power — using compile time. Set via: SET TIME HH:MM");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        Serial.println("[RTC] Ready");
    } else {
        Serial.println("[RTC] NOT FOUND — scheduling disabled");
    }

    wifiConnect();
    secClient.setInsecure();
    setupLoRa();
    loadAllSchedules();

    sendMainMenu();
    String boot = "🌱 Gateway Online v2.2";
    if (rtcOK) {
        DateTime now = rtc.now();
        char tb[8]; snprintf(tb, sizeof(tb), "%02d:%02d", now.hour(), now.minute());
        boot += "\n🕐 " + String(tb);
    }
    tgSend(boot);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
    receiveLoRa();      // HB + alerts only (does NOT consume ACKs)
    checkSchedules();   // RTC schedule trigger
    checkAutoOff();     // millis runtime enforcement
    checkHeartbeat();   // offline detection
    checkWiFi();        // reconnect watchdog
    handleTelegram();   // Telegram poll + dispatch
}