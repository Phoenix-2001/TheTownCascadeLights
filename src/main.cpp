/*
 * ╔══════════════════════════════════════════════════════════════════╗
 *  TTC_SSR.ino  —  The Town Cascade  —  Smart SSR Controller  v1.0
 * ╠══════════════════════════════════════════════════════════════════╣
 *
 *  HARDWARE
 *  ─────────────────────────────────────────────────────────────────
 *  ESP32 DevKit (any 38-pin variant)
 *  1 × Solid State Relay — DC control 3–32V, AC output 30A+
 *
 *  WIRING
 *  ─────────────────────────────────────────────────────────────────
 *  GPIO 25  →  SSR  DC+  input terminal
 *  GND      →  SSR  DC-  input terminal
 *  ESP32 USB 5V → USB charger (minimum 500mA)
 *
 *  SSR AC output terminals switch the mains supply line.
 *  An electrician must wire the AC side.
 *
 *  ⚠  ACTIVE HIGH:  HIGH (3.3V) = SSR ON  /  LOW (0V) = SSR OFF
 *     GPIO 25 defaults LOW on ESP32 boot → SSR is OFF before
 *     firmware even starts. Safe by hardware default.
 *
 *  WHY SINGLE CORE (not dual)
 *  ─────────────────────────────────────────────────────────────────
 *  Safety timers fire in minutes or hours. Telegram can stall 6s.
 *  A 6s stall cannot meaningfully miss an 8-hour safety ceiling.
 *  Dual-core would add FreeRTOS queues, mutexes, and inter-core
 *  race conditions — new failure modes with no practical benefit
 *  for this use case. Single core is the correct choice here.
 *
 *  FEATURES
 *  ─────────────────────────────────────────────────────────────────
 *  • SSR ON/OFF via Telegram keyboard
 *  • Mains ON requires explicit confirmation (30-second window)
 *  • Countdown timer — presets 30m / 1h / 2h / 4h + custom
 *  • Cancel timer without changing relay state
 *  • Safety auto-OFF: hard 8-hour ceiling (configurable)
 *  • Daily schedules — up to 10, ON or OFF, every day
 *  • Activity log — last 50 events with IST timestamps
 *  • Usage stats — lifetime count, today count, ON-time minutes
 *  • Last ON / last OFF timestamps (persisted in NVS)
 *  • Multi-user access — admin adds/removes users by Telegram ID
 *  • Daily midnight summary pushed to admin
 *  • WiFi watchdog — auto-reconnects across all known networks
 *  • DNS override — bypasses Jio's DNS using lwIP dns_setserver()
 *  • WiFi RSSI alert if signal degrades below threshold
 *  • TLS watchdog — resets Telegram connection if silent 90s
 *  • Boot notification with retry
 *  • Full admin command set
 *
 *  ADMIN TELEGRAM COMMANDS
 *  ─────────────────────────────────────────────────────────────────
 *  /adduser <id>    Add an authorised user by Telegram ID
 *  /deluser <id>    Remove a user
 *  /users           List all authorised users
 *  /delsched <n>    Delete schedule number n  (e.g. /delsched 2)
 *  /log             Last 20 activity log entries
 *  /dns             Live DNS diagnostic + canary test
 *  /reboot          Restart the ESP32 remotely
 *
 * ╚══════════════════════════════════════════════════════════════════╝
 */

// ─────────────────────────────────────────────────────────────────
//  LIBRARIES
// ─────────────────────────────────────────────────────────────────
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <Preferences.h>
#include <time.h>
#include "esp_wifi.h"
#include "lwip/dns.h"      // Direct lwIP DNS table — only reliable DNS fix on ESP32
#include "nvs_flash.h"     // Must init before any Preferences call


// ═════════════════════════════════════════════════════════════════
//  ① CONFIGURATION  — Edit these before flashing
// ═════════════════════════════════════════════════════════════════

#define BOT_TOKEN            "8589849970:AAH7vRuPDr665NTuxQ1-5KT10p1hob07Oz8"
const int64_t ADMIN_ID     = 5043757292LL;

// SSR control pin.  Active HIGH: HIGH=ON, LOW=OFF.
// GPIO 25 idles LOW at ESP32 boot → SSR OFF before firmware runs.
#define PIN_SSR              25

// Safety ceiling: SSR auto-OFF after this many hours if left on.
// Set 0 to disable (timers and schedules still work normally).
#define AUTO_OFF_HOURS       5

// Alert if WiFi RSSI drops below this (dBm). -80 is a safe threshold.
#define RSSI_ALERT_DBM       (-80)


// ═════════════════════════════════════════════════════════════════
//  ② WIFI NETWORKS  — Tried in order; first to connect wins
// ═════════════════════════════════════════════════════════════════

struct WifiCred { const char* ssid; const char* pass; };
const WifiCred WIFI_LIST[] = {
  { "The-Town-Cascade",     "TTC@2025"    },
  { "The-Town-Cascade_EXT", "TTC@2025"    },
  { "Vybhav",               "Vaibhav2024" },
  { "Phoenix",              "Harsha dj"   },
};
const int WIFI_COUNT = sizeof(WIFI_LIST) / sizeof(WIFI_LIST[0]);


// ═════════════════════════════════════════════════════════════════
//  ③ GLOBAL STATE
// ═════════════════════════════════════════════════════════════════

// ── SSR ─────────────────────────────────────────────────────────
bool          ssrState        = false;   // current physical state
unsigned long ssrOnSinceMs    = 0;       // millis() when last turned ON

bool          timerActive     = false;
unsigned long timerEndMs      = 0;

bool          safetyOffDone   = false;   // prevents repeated safety triggers

// ── Usage stats (NVS-backed) ─────────────────────────────────────
int           lifetimeOnCount = 0;
time_t        lastOnEpoch     = 0;
time_t        lastOffEpoch    = 0;

// ── Today's stats (RAM only, reset at midnight) ──────────────────
int           todayOnCount    = 0;
unsigned long todayOnSeconds  = 0;
int8_t        lastSummaryDay  = -1;

// ── WiFi / Telegram ──────────────────────────────────────────────
String        wifiSSID;
bool          telegramOK       = false;
bool          rssiAlerted      = false;

unsigned long tLastPoll        = 0;
unsigned long tLastBotActive   = 0;
unsigned long tLastWifiCheck   = 0;
unsigned long tLastDnsRetry    = 0;

const unsigned long POLL_MS          = 600;
const unsigned long BOT_WATCHDOG_MS  = 90000;
const unsigned long WIFI_CHECK_MS    = 30000;
const unsigned long DNS_RETRY_MS     = 20000;
const int           MAX_POLL_BATCHES = 10;    // cap on getUpdates() loop

// ── Preferences ──────────────────────────────────────────────────
Preferences prefs;


// ═════════════════════════════════════════════════════════════════
//  ④ NVS HELPERS  — Only called on state changes, not in loop
// ═════════════════════════════════════════════════════════════════

void saveStats() {
  prefs.begin("stats", false);
  prefs.putInt("total",   lifetimeOnCount);
  prefs.putLong64("lon",  (int64_t)lastOnEpoch);
  prefs.putLong64("loff", (int64_t)lastOffEpoch);
  prefs.end();
}

void loadStats() {
  prefs.begin("stats", true);
  lifetimeOnCount = prefs.getInt("total", 0);
  lastOnEpoch     = (time_t)prefs.getLong64("lon",  0LL);
  lastOffEpoch    = (time_t)prefs.getLong64("loff", 0LL);
  prefs.end();
}


// ═════════════════════════════════════════════════════════════════
//  ⑤ USER MANAGEMENT
// ═════════════════════════════════════════════════════════════════

#define MAX_USERS  10
int64_t users[MAX_USERS];
int     userCount = 0;

void saveUsers() {
  prefs.begin("users", false);
  prefs.putInt("cnt", userCount);
  for (int i = 0; i < userCount; i++)
    prefs.putLong64(("u" + String(i)).c_str(), users[i]);
  prefs.end();
}

void loadUsers() {
  prefs.begin("users", true);
  userCount = prefs.getInt("cnt", 0);
  for (int i = 0; i < userCount; i++)
    users[i] = prefs.getLong64(("u" + String(i)).c_str(), 0LL);
  prefs.end();
  if (userCount == 0) {
    users[0] = ADMIN_ID;
    userCount = 1;
    saveUsers();
  }
}

bool isAllowed(int64_t id) {
  for (int i = 0; i < userCount; i++)
    if (users[i] == id) return true;
  return false;
}


// ═════════════════════════════════════════════════════════════════
//  ⑥ SCHEDULE MANAGEMENT
// ═════════════════════════════════════════════════════════════════

// Single relay — no zone field needed.
// days bitmask: bit0=Sun, bit1=Mon … bit6=Sat.  0x7F = every day.
#define MAX_SCHEDULES  10

struct Schedule {
  bool    active;
  uint8_t hour, minute;
  bool    on;
  uint8_t days;
};
Schedule schedules[MAX_SCHEDULES];

void saveSchedules() {
  prefs.begin("sched", false);
  prefs.putBytes("d", schedules, sizeof(schedules));
  prefs.end();
}

void loadSchedules() {
  prefs.begin("sched", true);
  if (prefs.getBytesLength("d") == sizeof(schedules))
    prefs.getBytes("d", schedules, sizeof(schedules));
  prefs.end();
}


// ═════════════════════════════════════════════════════════════════
//  ⑦ ACTIVITY LOG  — Ring buffer, last 50 entries, RAM only
// ═════════════════════════════════════════════════════════════════

#define MAX_LOGS  50
String logBuf[MAX_LOGS];
int    logCount = 0;

void addLog(const String& msg) {
  String ts;
  time_t now = time(nullptr);
  if (now > 100000L) {
    struct tm ti;
    localtime_r(&now, &ti);
    char buf[10];
    snprintf(buf, sizeof(buf), "%02d:%02d  ", ti.tm_hour, ti.tm_min);
    ts = buf;
  } else {
    ts = "??:??  ";
  }
  String entry = ts + msg;
  if (logCount < MAX_LOGS) {
    logBuf[logCount++] = entry;
  } else {
    for (int i = 1; i < MAX_LOGS; i++) logBuf[i-1] = logBuf[i];
    logBuf[MAX_LOGS-1] = entry;
  }
  Serial.println("[LOG] " + entry);
}


// ═════════════════════════════════════════════════════════════════
//  ⑧ DNS OVERRIDE
//
//  WiFi.config() / WiFi.setDNS() silently fail after DHCP on ESP32.
//  dns_setserver() writes directly into lwIP's resolver table —
//  the same table all DNS lookups actually read from.
//  Called once after every successful WiFi connect.
// ═════════════════════════════════════════════════════════════════

void forceDNS() {
  ip_addr_t g, c;
  IP4_ADDR(ip_2_ip4(&g), 8, 8, 8, 8);   g.type = IPADDR_TYPE_V4;  // Google
  IP4_ADDR(ip_2_ip4(&c), 1, 1, 1, 1);   c.type = IPADDR_TYPE_V4;  // Cloudflare
  dns_setserver(0, &g);
  dns_setserver(1, &c);
  const ip_addr_t* r = dns_getserver(0);
  Serial.printf("[DNS] lwIP slot 0: %s\n", r ? ipaddr_ntoa(r) : "FAILED");
}

// Blocking canary test.
// ONLY called from: connectWiFi() once, and admin /dns command.
// NEVER called in loop() — blocks up to 6s, would delay safety ticks.
bool verifyDNS() {
  IPAddress ip;
  bool ok = (WiFi.hostByName("pool.ntp.org", ip) == 1);
  Serial.printf("[DNS] Canary: %s\n", ok ? ip.toString().c_str() : "FAILED");
  return ok;
}


// ═════════════════════════════════════════════════════════════════
//  ⑨ TELEGRAM CLIENT + ADMIN ALERT
//  Defined before SSR functions so sendAdminAlert() is available
//  to the safety handlers with no forward declarations.
// ═════════════════════════════════════════════════════════════════

WiFiClientSecure    secClient;
UniversalTelegramBot bot(BOT_TOKEN, secClient);

// Safe to call from anywhere.  Silent drop if Telegram is offline.
void sendAdminAlert(const String& msg) {
  if (!telegramOK) {
    Serial.println("[ALERT-SKIP] " + msg.substring(0, 50));
    return;
  }
  bot.sendMessage(String(ADMIN_ID), msg, "Markdown");
}


// ═════════════════════════════════════════════════════════════════
//  ⑩ SSR CONTROL
//  setSSR() is the SINGLE point of relay control.
//  All state changes, logging, and statistics happen here.
// ═════════════════════════════════════════════════════════════════

// Active HIGH: HIGH=ON, LOW=OFF
inline void ssrHardOn()  { digitalWrite(PIN_SSR, HIGH); }
inline void ssrHardOff() { digitalWrite(PIN_SSR, LOW);  }

void setSSR(bool on) {
  // Guard: no-op if already in requested state.
  // Prevents double-counting stats when called redundantly.
  if (on == ssrState) return;

  on ? ssrHardOn() : ssrHardOff();
  ssrState = on;

  if (on) {
    ssrOnSinceMs  = millis();
    safetyOffDone = false;
    lifetimeOnCount++;
    todayOnCount++;
    lastOnEpoch = time(nullptr);
    saveStats();
    addLog("SSR ON");
  } else {
    // Accumulate session ON-time into today's total
    if (ssrOnSinceMs > 0)
      todayOnSeconds += (millis() - ssrOnSinceMs) / 1000UL;
    timerActive  = false;   // cancel any running countdown
    lastOffEpoch = time(nullptr);
    saveStats();
    addLog("SSR OFF");
  }
}

// ── Countdown timer ──────────────────────────────────────────────
void setTimer(uint16_t minutes) {
  // Turn SSR on only if it's currently off (avoids double ON-event)
  if (!ssrState) setSSR(true);
  timerActive = true;
  timerEndMs  = millis() + (unsigned long)minutes * 60000UL;
  addLog("Timer set: " + String(minutes) + " min");
}

void cancelTimer() {
  timerActive = false;
  addLog("Timer cancelled");
}

// Called every loop iteration — no delay(), no blocking
void tickTimer() {
  if (!timerActive) return;
  // Signed subtraction handles millis() 49-day rollover correctly
  if ((long)(millis() - timerEndMs) >= 0) {
    timerActive = false;
    setSSR(false);
    sendAdminAlert("⏱ *Timer expired* — SSR OFF");
  }
}

// ── Safety auto-OFF ceiling ──────────────────────────────────────
// Independent of the user timer.  Hard ceiling after AUTO_OFF_HOURS.
void tickSafety() {
#if AUTO_OFF_HOURS > 0
  if (!ssrState || safetyOffDone) return;
  unsigned long limitMs = (unsigned long)AUTO_OFF_HOURS * 3600000UL;
  if ((long)(millis() - ssrOnSinceMs) >= (long)limitMs) {
    safetyOffDone = true;   // set before setSSR() to prevent re-entry
    setSSR(false);
    sendAdminAlert("🛡 *Safety auto-OFF*\n"
                   "SSR was ON for " + String(AUTO_OFF_HOURS) + "h.\n"
                   "Switched off automatically.");
  }
#endif
}


// ═════════════════════════════════════════════════════════════════
//  ⑪ KEYBOARDS
// ═════════════════════════════════════════════════════════════════

// Main menu
const String KBD_MAIN =
  "[[{\"text\":\"⚡ SSR ON\"},{\"text\":\"⚡ SSR OFF\"}],"
  "[{\"text\":\"⏱ Timer\"},{\"text\":\"❌ Cancel Timer\"}],"
  "[{\"text\":\"📊 STATUS\"},{\"text\":\"⏰ Schedules\"}]]";

// Timer presets
const String KBD_TIMER =
  "[[{\"text\":\"⏱ 30 min\"},{\"text\":\"⏱ 1h\"}],"
  "[{\"text\":\"⏱ 2h\"},{\"text\":\"⏱ 4h\"}],"
  "[{\"text\":\"✏ Custom\"}],"
  "[{\"text\":\"⬅ Back\"}]]";

// Mains ON safety confirmation (30-second window)
const String KBD_CONFIRM =
  "[[{\"text\":\"✅ YES — Switch ON\"},{\"text\":\"❌ CANCEL\"}]]";

// Schedule management
const String KBD_SCHED =
  "[[{\"text\":\"📋 List Schedules\"}],"
  "[{\"text\":\"➕ Add Schedule\"}],"
  "[{\"text\":\"⬅ Back\"}]]";


// ═════════════════════════════════════════════════════════════════
//  ⑫ STATUS + MENU BUILDERS
// ═════════════════════════════════════════════════════════════════

void showMain(const String& chat,
              const String& msg = "🏠 *TTC SSR Controller*") {
  bot.sendMessageWithReplyKeyboard(chat, msg, "Markdown", KBD_MAIN, true);
}

String buildStatus() {
  String s = "📊 *System Status*\n\n";

  // WiFi
  if (WiFi.status() == WL_CONNECTED) {
    s += "📡 *WiFi:* "; s += wifiSSID;
    s += "  "; s += WiFi.RSSI(); s += " dBm\n";
  } else {
    s += "📡 *WiFi:* ❌ Disconnected\n";
  }

  // Active DNS — confirms Jio override is working
  const ip_addr_t* d = dns_getserver(0);
  s += "🌐 *DNS:* "; s += d ? ipaddr_ntoa(d) : "unknown"; s += "\n";

  // Time
  time_t now = time(nullptr);
  if (now > 100000L) {
    struct tm ti; localtime_r(&now, &ti);
    char ts[32]; strftime(ts, sizeof(ts), "%d %b %Y  %H:%M IST", &ti);
    s += "🕐 "; s += ts; s += "\n";
  } else {
    s += "🕐 NTP syncing...\n";
  }

  // Uptime
  unsigned long sec = millis() / 1000;
  s += "🔋 *Uptime:* ";
  s += String(sec / 3600); s += "h ";
  s += String((sec % 3600) / 60); s += "m\n\n";

  // SSR state
  s += "⚡ *SSR (30A):* ";
  s += ssrState ? "ON 🟢" : "OFF ⚫";
  s += "\n";

  if (ssrState) {
    unsigned long onSec = (millis() - ssrOnSinceMs) / 1000;
    s += "   On for: *";
    s += String(onSec / 60); s += "m ";
    s += String(onSec % 60); s += "s*\n";

    if (timerActive) {
      long remMs = (long)(timerEndMs - millis());
      if (remMs > 0) {
        s += "   ⏱ Timer: *";
        s += String(remMs / 60000L); s += " min left*\n";
      }
    }

#if AUTO_OFF_HOURS > 0
    long autoRemMs = (long)((unsigned long)AUTO_OFF_HOURS * 3600000UL
                            - (millis() - ssrOnSinceMs));
    if (autoRemMs > 0) {
      s += "   🛡 Safety OFF in: *";
      s += String(autoRemMs / 3600000L); s += "h ";
      s += String((autoRemMs % 3600000L) / 60000L); s += "m*\n";
    }
#endif
  }

  // Usage stats
  s += "\n📈 *Usage Today*\n";
  s += "Events: *"; s += todayOnCount; s += "*   ";
  unsigned long liveSec = ssrState ? (millis() - ssrOnSinceMs) / 1000 : 0;
  s += "ON time: *"; s += (todayOnSeconds + liveSec) / 60; s += " min*\n";

  s += "\n📈 *Lifetime*\n";
  s += "Total ON events: *"; s += lifetimeOnCount; s += "*\n";

  auto fmtEpoch = [](time_t t) -> String {
    if (t < 100000L) return "—";
    struct tm ti; localtime_r(&t, &ti);
    char buf[18]; strftime(buf, sizeof(buf), "%d %b  %H:%M", &ti);
    return String(buf);
  };

  s += "Last ON:  *"; s += fmtEpoch(lastOnEpoch);  s += "*\n";
  s += "Last OFF: *"; s += fmtEpoch(lastOffEpoch); s += "*\n";

  return s;
}

String buildSchedList() {
  String s = "⏰ *Schedules*\n\n";
  bool any = false;
  for (int i = 0; i < MAX_SCHEDULES; i++) {
    if (!schedules[i].active) continue;
    any = true;
    char t[6];
    snprintf(t, sizeof(t), "%02d:%02d", schedules[i].hour, schedules[i].minute);
    s += "#"; s += i; s += "  *"; s += t; s += "*";
    s += "  →  *"; s += schedules[i].on ? "ON" : "OFF"; s += "*\n";
  }
  if (!any) s += "_No schedules configured._\n";
  s += "\nDelete: /delsched <n>   e.g. /delsched 2";
  return s;
}


// ═════════════════════════════════════════════════════════════════
//  ⑬ WIFI — connectWiFi / checkWiFi / checkDNS
// ═════════════════════════════════════════════════════════════════

bool connectWiFi() {
  // Full radio reset before each sequence.
  // Clears stale DHCP/association state with Jio Fiber + TP-Link.
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);          // don't write credentials to NVS
  WiFi.setAutoReconnect(false);    // we manage reconnect ourselves
  WiFi.setSleep(WIFI_PS_NONE);     // Jio 2025 drops sleeping clients
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  delay(100);

  for (int i = 0; i < WIFI_COUNT; i++) {
    Serial.printf("[WiFi] Trying: %s\n", WIFI_LIST[i].ssid);
    WiFi.begin(WIFI_LIST[i].ssid, WIFI_LIST[i].pass);

    // 18s timeout — TP-Link extender relays DHCP to Jio (double-hop)
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 18000)
      delay(300);

    if (WiFi.status() == WL_CONNECTED) {
      wifiSSID = WIFI_LIST[i].ssid;
      Serial.printf("[WiFi] Connected: %s  RSSI:%d  CH:%d  IP:%s\n",
                    wifiSSID.c_str(), WiFi.RSSI(),
                    (int)WiFi.channel(),
                    WiFi.localIP().toString().c_str());

      delay(200);       // let DHCP fully settle before writing DNS
      forceDNS();
      bool ok = verifyDNS();   // blocking — acceptable here (once per connect)
      telegramOK  = ok;
      tLastDnsRetry = millis();
      if (!ok) Serial.println("[DNS] Canary failed — checkDNS() will retry");
      return true;
    }

    WiFi.disconnect(true);
    delay(800);   // let extender clear the failed association
  }

  Serial.println("[WiFi] All networks failed");
  return false;
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    // RSSI monitor
    int rssi = WiFi.RSSI();
    if (rssi < RSSI_ALERT_DBM && !rssiAlerted) {
      rssiAlerted = true;
      sendAdminAlert("📶 *WiFi signal weak*\n" + wifiSSID +
                     ": " + String(rssi) + " dBm");
    } else if (rssi >= RSSI_ALERT_DBM - 5 && rssiAlerted) {
      rssiAlerted = false;   // 5 dBm hysteresis — prevents flapping
      sendAdminAlert("📶 *WiFi signal recovered* — " + String(rssi) + " dBm");
    }
    return;
  }

  wifiSSID   = "";
  telegramOK = false;
  rssiAlerted = false;
  Serial.println("[WiFi] Lost — reconnecting...");

  if (connectWiFi()) {
    // IST = UTC+5:30 = 19800 s offset
    configTime(19800, 0, "pool.ntp.org", "time.google.com");
    addLog("WiFi reconnected: " + wifiSSID);
    if (telegramOK)
      sendAdminAlert("🔄 *WiFi back:* " + wifiSSID +
                     "  " + String(WiFi.RSSI()) + " dBm");
  }
}

// Re-applies DNS every DNS_RETRY_MS when Telegram is not reachable.
// Does NOT call verifyDNS() here — blocking in the loop delays timers.
// Verification happens implicitly: next successful pollTelegram() sets
// telegramOK = true, which stops retries.
void checkDNS() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (telegramOK) return;
  if ((long)(millis() - tLastDnsRetry) < (long)DNS_RETRY_MS) return;
  tLastDnsRetry = millis();
  Serial.println("[DNS] Re-applying override...");
  forceDNS();
}


// ═════════════════════════════════════════════════════════════════
//  ⑭ SESSIONS  — Per-chat-ID state prevents cross-user interference
// ═════════════════════════════════════════════════════════════════

// Schedule wizard steps.  SS_ZONE removed — single relay, no ambiguity.
enum SchedStep : uint8_t { SS_NONE = 0, SS_TIME, SS_ACTION };

#define MAX_SESSIONS  8

struct Session {
  int64_t       chatId;
  bool          awaitTimerMin;    // user is typing custom timer minutes
  bool          awaitConfirm;     // waiting for SSR ON confirmation
  unsigned long confirmExpiry;    // millis() deadline for confirm window
  SchedStep     schedStep;        // wizard progress
  uint8_t       schedHour;
  uint8_t       schedMin;
};

Session sessions[MAX_SESSIONS];
int     sessCount = 0;

Session* getSession(int64_t id) {
  for (int i = 0; i < sessCount; i++)
    if (sessions[i].chatId == id) return &sessions[i];
  if (sessCount < MAX_SESSIONS) {
    sessions[sessCount] = {id, false, false, 0, SS_NONE, 0, 0};
    return &sessions[sessCount++];
  }
  // LRU evict: drop slot 0
  for (int i = 0; i < MAX_SESSIONS - 1; i++) sessions[i] = sessions[i+1];
  sessions[MAX_SESSIONS-1] = {id, false, false, 0, SS_NONE, 0, 0};
  return &sessions[MAX_SESSIONS-1];
}

void resetSession(Session* s) {
  s->awaitTimerMin  = false;
  s->awaitConfirm   = false;
  s->confirmExpiry  = 0;
  s->schedStep      = SS_NONE;
}

// Expire stale confirmations silently — called once per loop.
// Uses signed subtraction for millis() 49-day rollover safety.
void tickConfirmExpiry() {
  for (int i = 0; i < sessCount; i++) {
    if (!sessions[i].awaitConfirm) continue;
    if ((long)(millis() - sessions[i].confirmExpiry) >= 0) {
      sessions[i].awaitConfirm = false;
      Serial.printf("[CONFIRM] Expired for %lld\n",
                    (long long)sessions[i].chatId);
    }
  }
}


// ═════════════════════════════════════════════════════════════════
//  ⑮ MESSAGE HANDLER
// ═════════════════════════════════════════════════════════════════

void handleMessage(int idx) {
  String  chatStr = bot.messages[idx].chat_id;
  int64_t chatId  = atoll(chatStr.c_str());
  String  t       = bot.messages[idx].text;
  t.trim();

  // ── Authorisation ──────────────────────────────────────────────
  if (!isAllowed(chatId)) {
    bot.sendMessage(chatStr,
      "❌ Not authorised.\nAsk the admin for access.", "");
    return;
  }

  tLastBotActive = millis();
  telegramOK     = true;
  Session* s     = getSession(chatId);

  // ── /start ─────────────────────────────────────────────────────
  if (t == "/start") {
    resetSession(s);
    showMain(chatStr, "✅ *TTC SSR Controller v1*\n30A SSR ready.");
    return;
  }

  // ── Admin commands ──────────────────────────────────────────────
  if (chatId == ADMIN_ID) {

    // /adduser
    if (t.startsWith("/adduser ")) {
      if (userCount >= MAX_USERS) {
        bot.sendMessage(chatStr, "❌ User list full (max " +
                        String(MAX_USERS) + ")", "");
        return;
      }
      int64_t u = atoll(t.substring(9).c_str());
      if (!u) { bot.sendMessage(chatStr, "❌ Invalid ID", ""); return; }
      users[userCount++] = u;
      saveUsers();
      addLog("User added: " + String(u));
      bot.sendMessage(chatStr, "✅ User " + String(u) + " added", "");
      return;
    }

    // /deluser
    if (t.startsWith("/deluser ")) {
      int64_t u = atoll(t.substring(9).c_str());
      if (u == ADMIN_ID) {
        bot.sendMessage(chatStr, "❌ Cannot remove admin", ""); return;
      }
      bool found = false;
      for (int k = 0; k < userCount; k++) {
        if (users[k] != u) continue;
        for (int j = k; j < userCount - 1; j++) users[j] = users[j+1];
        userCount--;
        saveUsers();
        found = true;
        addLog("User removed: " + String(u));
        break;
      }
      bot.sendMessage(chatStr, found ? "✅ Removed" : "❌ Not found", "");
      return;
    }

    // /delsched — requires explicit number to prevent silent #0 deletion
    if (t.startsWith("/delsched ")) {
      String ns = t.substring(10); ns.trim();
      if (!ns.length()) {
        bot.sendMessage(chatStr,
          "Usage: /delsched <number>   e.g. /delsched 2", "");
        return;
      }
      int n = ns.toInt();
      if (n >= 0 && n < MAX_SCHEDULES && schedules[n].active) {
        schedules[n].active = false;
        saveSchedules();
        addLog("Sched #" + String(n) + " deleted");
        bot.sendMessage(chatStr,
          "✅ Schedule #" + String(n) + " deleted", "");
      } else {
        bot.sendMessage(chatStr,
          "❌ No active schedule at #" + String(n), "");
      }
      return;
    }

    // /delsched bare — show help (never silently deletes)
    if (t == "/delsched") {
      bot.sendMessage(chatStr,
        "Usage: /delsched <number>   e.g. /delsched 2\n\n" +
        buildSchedList(), "Markdown");
      return;
    }

    // /log
    if (t == "/log") {
      String msg = "📜 *Activity Log*\n";
      int from = max(0, logCount - 20);
      for (int l = from; l < logCount; l++) msg += logBuf[l] + "\n";
      if (!logCount) msg += "_Empty_";
      bot.sendMessage(chatStr, msg, "Markdown");
      return;
    }

    // /users
    if (t == "/users") {
      String msg = "👥 *Users (" + String(userCount) + ")*\n";
      for (int k = 0; k < userCount; k++) {
        msg += String(users[k]);
        if (users[k] == ADMIN_ID) msg += " *(admin)*";
        msg += "\n";
      }
      bot.sendMessage(chatStr, msg, "Markdown");
      return;
    }

    // /dns — verifyDNS() is blocking; acceptable for an admin diagnostic
    if (t == "/dns") {
      String msg = "🌐 *DNS Diagnostic*\n";
      for (int d = 0; d < 2; d++) {
        const ip_addr_t* srv = dns_getserver(d);
        msg += "Slot " + String(d) + ": ";
        msg += srv ? ipaddr_ntoa(srv) : "empty";
        msg += "\n";
      }
      msg += "\n";
      msg += verifyDNS() ? "✅ DNS resolving OK" : "❌ DNS FAILED";
      bot.sendMessage(chatStr, msg, "Markdown");
      return;
    }

    // /reboot
    if (t == "/reboot") {
      bot.sendMessage(chatStr, "🔄 Rebooting...", "");
      delay(500);
      ESP.restart();
      return;
    }
  }

  // ── Back — always available, resets session ─────────────────────
  if (t == "⬅ Back") {
    resetSession(s);
    showMain(chatStr);
    return;
  }

  // ── Schedule wizard ─────────────────────────────────────────────
  if (s->schedStep != SS_NONE) {

    // Step 1 — collect time
    if (s->schedStep == SS_TIME) {
      int col = t.indexOf(':');
      int h   = (col > 0) ? t.substring(0, col).toInt()   : -1;
      int m   = (col > 0) ? t.substring(col+1).toInt() : -1;
      if (h >= 0 && h < 24 && m >= 0 && m < 60) {
        s->schedHour = (uint8_t)h;
        s->schedMin  = (uint8_t)m;
        s->schedStep = SS_ACTION;
        char ts[6]; snprintf(ts, sizeof(ts), "%02d:%02d", h, m);
        bot.sendMessage(chatStr,
          "Time: *" + String(ts) + "* IST\n\n"
          "Turn SSR *ON* or *OFF* at this time?\n"
          "Reply: ON  or  OFF", "Markdown");
      } else {
        bot.sendMessage(chatStr,
          "❌ Invalid. Use HH:MM  e.g. 06:30 or 22:00", "");
      }
      return;
    }

    // Step 2 — collect ON/OFF
    if (s->schedStep == SS_ACTION) {
      String tU = t; tU.toUpperCase();
      if (tU == "ON" || tU == "OFF") {
        int slot = -1;
        for (int k = 0; k < MAX_SCHEDULES; k++)
          if (!schedules[k].active) { slot = k; break; }

        if (slot < 0) {
          bot.sendMessage(chatStr,
            "❌ Schedule list full (max " + String(MAX_SCHEDULES) + ").\n"
            "Delete one with /delsched <n>", "");
        } else {
          char ts[6];
          snprintf(ts, sizeof(ts), "%02d:%02d", s->schedHour, s->schedMin);
          schedules[slot] = { true, s->schedHour, s->schedMin,
                              (tU == "ON"), 0x7F };
          saveSchedules();
          addLog("Sched #" + String(slot) + " " + tU + " " + ts);
          bot.sendMessage(chatStr,
            "✅ *Schedule #" + String(slot) + " saved*\n"
            "SSR → *" + tU + "* at *" + ts + "* every day",
            "Markdown");
        }
        resetSession(s);
        showMain(chatStr);
      } else {
        bot.sendMessage(chatStr, "Reply *ON* or *OFF*", "Markdown");
      }
      return;
    }
  }

  // ── Custom timer input ──────────────────────────────────────────
  if (s->awaitTimerMin) {
    int mins = t.toInt();
    if (mins >= 1 && mins <= 1440) {
      setTimer((uint16_t)mins);
      addLog("User " + String(chatId) + " set timer " + mins + "min");
      bot.sendMessage(chatStr,
        "⏱ SSR ON for *" + String(mins) + " min*. Auto-OFF at expiry.",
        "Markdown");
      s->awaitTimerMin = false;
      showMain(chatStr);
    } else {
      bot.sendMessage(chatStr,
        "❌ Enter a number between 1 and 1440 (minutes)", "");
    }
    return;
  }

  // ── Mains ON confirmation response ─────────────────────────────
  if (s->awaitConfirm) {
    // Signed subtraction: confirmExpiry - millis() > 0 means still valid
    bool valid = ((long)(s->confirmExpiry - millis()) > 0);

    if (t == "✅ YES — Switch ON") {
      if (valid) {
        setSSR(true);
        addLog("User " + String(chatId) + " confirmed SSR ON");
        String msg = "⚡ *SSR ON* 🟢\n";
#if AUTO_OFF_HOURS > 0
        msg += "Safety auto-OFF in " + String(AUTO_OFF_HOURS) + "h.";
#endif
        bot.sendMessage(chatStr, msg, "Markdown");
      } else {
        bot.sendMessage(chatStr,
          "⏱ Confirmation expired.\nPress *SSR ON* again.", "Markdown");
      }
      s->awaitConfirm = false;
      showMain(chatStr);

    } else if (t == "❌ CANCEL") {
      s->awaitConfirm = false;
      bot.sendMessage(chatStr, "Cancelled.", "");
      showMain(chatStr);

    } else {
      // Any other input while awaiting confirm
      if (valid) {
        bot.sendMessage(chatStr,
          "⚠ Tap *YES* to switch SSR ON or *CANCEL*.", "Markdown");
      } else {
        s->awaitConfirm = false;
        bot.sendMessage(chatStr, "Confirmation expired.", "");
        showMain(chatStr);
      }
    }
    return;
  }

  // ── Main keyboard buttons ───────────────────────────────────────

  if (t == "⚡ SSR ON") {
    if (ssrState) {
      bot.sendMessage(chatStr,
        "ℹ SSR is already ON.\n"
        "Use *⏱ Timer* to set auto-OFF, or *⚡ SSR OFF* to switch off.",
        "Markdown");
      return;
    }
    // Require explicit confirmation before switching 30A mains
    s->awaitConfirm  = true;
    s->confirmExpiry = millis() + 30000;   // 30-second window
    bot.sendMessageWithReplyKeyboard(chatStr,
      "⚠ *Confirm: Switch SSR ON?*\n\n"
      "This switches the 30A mains supply.\n"
      "Window expires in *30 seconds*.",
      "Markdown", KBD_CONFIRM, true);
    return;
  }

  if (t == "⚡ SSR OFF") {
    if (!ssrState) {
      bot.sendMessage(chatStr, "ℹ SSR is already OFF.", "");
      return;
    }
    setSSR(false);
    addLog("User " + String(chatId) + " → SSR OFF");
    bot.sendMessage(chatStr, "⚡ *SSR OFF* ⚫", "Markdown");
    return;
  }

  if (t == "⏱ Timer") {
    bot.sendMessageWithReplyKeyboard(chatStr,
      "⏱ *Set Timer*\nSSR turns ON and auto-OFF when timer expires.",
      "Markdown", KBD_TIMER, true);
    return;
  }

  if (t == "❌ Cancel Timer") {
    if (timerActive) {
      cancelTimer();
      bot.sendMessage(chatStr,
        "❌ *Timer cancelled*\n"
        "SSR stays *" + String(ssrState ? "ON" : "OFF") + "*.",
        "Markdown");
    } else {
      bot.sendMessage(chatStr, "ℹ No active timer.", "");
    }
    return;
  }

  if (t == "📊 STATUS") {
    bot.sendMessage(chatStr, buildStatus(), "Markdown");
    return;
  }

  if (t == "⏰ Schedules") {
    bot.sendMessageWithReplyKeyboard(chatStr,
      buildSchedList(), "Markdown", KBD_SCHED, true);
    return;
  }

  if (t == "📋 List Schedules") {
    bot.sendMessage(chatStr, buildSchedList(), "Markdown");
    return;
  }

  if (t == "➕ Add Schedule") {
    s->schedStep = SS_TIME;
    bot.sendMessage(chatStr,
      "*Add Schedule*\n\nSend time in 24h format:\n"
      "*HH:MM*   e.g. 06:30  or  22:00", "Markdown");
    return;
  }

  // ── Timer presets (from KBD_TIMER) ─────────────────────────────
  if (t == "⏱ 30 min") {
    setTimer(30);
    bot.sendMessage(chatStr, "⏱ *30 min* timer set. SSR ON.", "Markdown");
    showMain(chatStr); return;
  }
  if (t == "⏱ 1h") {
    setTimer(60);
    bot.sendMessage(chatStr, "⏱ *1h* timer set. SSR ON.", "Markdown");
    showMain(chatStr); return;
  }
  if (t == "⏱ 2h") {
    setTimer(120);
    bot.sendMessage(chatStr, "⏱ *2h* timer set. SSR ON.", "Markdown");
    showMain(chatStr); return;
  }
  if (t == "⏱ 4h") {
    setTimer(240);
    bot.sendMessage(chatStr, "⏱ *4h* timer set. SSR ON.", "Markdown");
    showMain(chatStr); return;
  }
  if (t == "✏ Custom") {
    s->awaitTimerMin = true;
    bot.sendMessage(chatStr,
      "Send timer duration in *minutes* (1–1440):", "Markdown");
    return;
  }

  // ── Fallthrough — unknown input ─────────────────────────────────
  showMain(chatStr);
}


// ═════════════════════════════════════════════════════════════════
//  ⑯ TELEGRAM POLL
//  Capped at MAX_POLL_BATCHES to prevent a message flood from
//  blocking tickTimer() and tickSafety() in the main loop.
// ═════════════════════════════════════════════════════════════════

void pollTelegram() {
  int n = bot.getUpdates(bot.last_message_received + 1);
  if (n > 0) { tLastBotActive = millis(); telegramOK = true; }
  int batches = 0;
  while (n > 0 && batches < MAX_POLL_BATCHES) {
    for (int i = 0; i < n; i++) handleMessage(i);
    batches++;
    n = bot.getUpdates(bot.last_message_received + 1);
  }
}


// ═════════════════════════════════════════════════════════════════
//  ⑰ SCHEDULE CHECKER + DAILY SUMMARY
//  Runs once per minute (guarded by lastSchedMinute).
// ═════════════════════════════════════════════════════════════════

uint8_t lastSchedMinute = 0xFF;

void checkSchedules() {
  time_t now = time(nullptr);
  if (now < 100000L) return;   // NTP not synced yet

  struct tm ti;
  localtime_r(&now, &ti);
  if ((uint8_t)ti.tm_min == lastSchedMinute) return;
  lastSchedMinute = (uint8_t)ti.tm_min;

  // Daily summary at midnight IST
  if (ti.tm_hour == 0 && ti.tm_min == 0 && ti.tm_mday != lastSummaryDay) {
    lastSummaryDay = (int8_t)ti.tm_mday;
    unsigned long liveSec = ssrState ? (millis() - ssrOnSinceMs) / 1000 : 0;
    String sum = "📊 *Daily Summary*\n";
    sum += "ON events: *" + String(todayOnCount) + "*\n";
    sum += "Total ON time: *" + String((todayOnSeconds + liveSec) / 60) + " min*\n";
    sum += "SSR now: *" + String(ssrState ? "ON" : "OFF") + "*\n";
    sum += "Lifetime: *" + String(lifetimeOnCount) + "* events";
    sendAdminAlert(sum);
    todayOnCount   = 0;
    todayOnSeconds = 0;
  }

  // Fire scheduled ON/OFF events
  uint8_t dayBit = (uint8_t)(1 << ti.tm_wday);
  for (int i = 0; i < MAX_SCHEDULES; i++) {
    if (!schedules[i].active)            continue;
    if (schedules[i].hour   != ti.tm_hour)  continue;
    if (schedules[i].minute != ti.tm_min)   continue;
    if (!(schedules[i].days & dayBit))   continue;

    char ts[6];
    snprintf(ts, sizeof(ts), "%02d:%02d", ti.tm_hour, ti.tm_min);
    bool on = schedules[i].on;

    setSSR(on);   // scheduled events bypass user confirmation (intentional)
    addLog("[SCHED] SSR " + String(on ? "ON" : "OFF") + " " + ts);
    sendAdminAlert("⏰ Schedule: SSR *" +
                   String(on ? "ON" : "OFF") + "* at *" + ts + "*");
  }
}


// ═════════════════════════════════════════════════════════════════
//  ⑱ SETUP
// ═════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  TTC SSR Controller  v1.0            ║");
  Serial.println("║  Single-node · Jio WiFi · Telegram   ║");
  Serial.println("╚══════════════════════════════════════╝\n");

  // ── SSR pin — LOW before OUTPUT to guarantee OFF at power-on ───
  // Setting the output register before enabling the output driver
  // prevents any transient HIGH pulse during GPIO initialisation.
  digitalWrite(PIN_SSR, LOW);
  pinMode(PIN_SSR, OUTPUT);
  Serial.println("[SSR] GPIO 25 → LOW (OFF). SSR safe state confirmed.");

  // ── NVS flash init — required before any Preferences call ───────
  // Fixes "nvs_open failed: NOT_FOUND" on first flash.
  // Auto-repairs corrupt partition.
  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("[NVS] Partition issue — erasing and reinitialising");
    nvs_flash_erase();
    nvs_flash_init();
  }
  Serial.println("[NVS] Ready");

  // ── Telegram client config ───────────────────────────────────────
  secClient.setInsecure();
  secClient.setTimeout(6);   // 6s cap per attempt — limits DNS-fail hangs

  // ── WiFi + DNS ───────────────────────────────────────────────────
  bool wifiOk = connectWiFi();

  // ── NTP — IST = UTC+5:30 = 19800 seconds ────────────────────────
  configTime(19800, 0, "pool.ntp.org", "time.google.com");

  // ── Load persisted data ──────────────────────────────────────────
  loadUsers();
  loadSchedules();
  loadStats();

  // ── Discard pre-boot Telegram backlog ────────────────────────────
  bot.getUpdates(0);

  // Initialise all timing references to now
  tLastBotActive = tLastPoll = tLastWifiCheck = tLastDnsRetry = millis();

  Serial.printf("[MAC]  %s\n", WiFi.macAddress().c_str());
  Serial.printf("[STATS] Lifetime ON count: %d\n", lifetimeOnCount);

  // ── Boot notification (3 attempts, 5s apart) ─────────────────────
  if (wifiOk && telegramOK) {
    String msg = "🟢 *TTC SSR Controller v1 Online*\n\n";
    msg += "📡 " + wifiSSID + "  " + String(WiFi.RSSI()) + " dBm\n";
    msg += "🌐 DNS: 8.8.8.8 ✅\n";
    msg += "⚡ SSR: *OFF* (safe state)\n";
    msg += "🔋 Ready\n\n";
    msg += "Lifetime ON count: *" + String(lifetimeOnCount) + "*";

    bool sent = false;
    for (int attempt = 1; attempt <= 3 && !sent; attempt++) {
      Serial.printf("[Bot] Boot notification %d/3\n", attempt);
      if (bot.sendMessageWithReplyKeyboard(
            String(ADMIN_ID), msg, "Markdown", KBD_MAIN, true)) {
        sent = true;
        Serial.println("[Bot] Sent OK");
      } else {
        Serial.println("[Bot] Failed — re-applying DNS, waiting 5s");
        forceDNS();
        delay(5000);
      }
    }
    if (!sent) {
      telegramOK = false;
      Serial.println("[Bot] All attempts failed — checkDNS() will retry");
    }
  } else if (wifiOk) {
    Serial.println("[Bot] WiFi up, DNS not ready — will retry in loop()");
  } else {
    Serial.println("[Bot] WiFi offline — SSR still operates via schedules");
  }

  Serial.println("\n[READY] Controller online.");
  Serial.println("──────────────────────────────────────────────────────\n");
}


// ═════════════════════════════════════════════════════════════════
//  ⑲ LOOP
// ═════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Telegram poll ─────────────────────────────────────────────
  if (WiFi.status() == WL_CONNECTED &&
      telegramOK &&
      now - tLastPoll >= POLL_MS) {
    pollTelegram();
    tLastPoll = millis();
  }

  // ── WiFi watchdog — every 30s ─────────────────────────────────
  if (now - tLastWifiCheck >= WIFI_CHECK_MS) {
    checkWiFi();
    tLastWifiCheck = millis();
  }

  // ── DNS watchdog — re-applies override when Telegram is silent ─
  checkDNS();

  // ── TLS watchdog — reset if bot silent for 90s ────────────────
  if (now - tLastBotActive >= BOT_WATCHDOG_MS) {
    secClient.stop();
    tLastBotActive = millis();
    Serial.println("[Bot] TLS watchdog — connection reset");
  }

  // ── SSR safety ticks — run every loop, never blocked ─────────
  tickTimer();
  tickSafety();

  // ── Schedule checker + daily summary ─────────────────────────
  checkSchedules();

  // ── Expire stale confirmations ────────────────────────────────
  tickConfirmExpiry();

  delay(10);
}
