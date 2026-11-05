/*
 Power Notifier + Battery Monitor (ESP8266 / Wemos D1 mini)
 - INA226 + external shunt 200A/75mV (R=0.000375)
 - AC detect via opto: D7 (GPIO13), LOW = AC present
 - Telegram: AC lost / AC restored + /status command
 - Real time (NTP) when WiFi turns ON; keeps time while WiFi OFF
 - WiFi policy:
   - OFF normally (battery saver)
   - ON to send event
   - ON once per N minutes to poll bot for /status
 - Encoder UI:
   - Long press: enter battery select
   - Rotate: pick battery type
   - Long press in select: save battery type
   - Short press in monitor: mute warnings
   - Rotation in monitor: switch LCD pages

 LCD1602 I2C: SDA=D2(GPIO4), SCL=D1(GPIO5)
*/

#include <Wire.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <math.h>

// ========================= USER SETTINGS =========================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASS";
const char* TG_BOT_TOKEN = "YOUR_TELEGRAM_BOT_TOKEN";
const char* TG_CHAT_ID = "YOUR_CHAT_ID";


// ---- CHANGE HERE: how often to poll bot for /status (minutes) ----
const uint32_t BOT_POLL_EVERY_MIN = 3; // як часто бот перевіряє пул запитів з особистого чату в Telegram
// ---------------------------------------------------------------

const uint32_t AC_NOTIFY_DELAY_MS = 1UL * 60UL * 1000UL;  // 1 хв (реконект роутера при відключені мережі)

const float ETA_CHARGE = 0.83f;                           // ККД заряду (0.82..0.85)

// ========================= SYSTEM / BATTERY SPECS =========================
// Загальний опір (проводів, клем, шунта, переходів) для компенсації просадки під навантаженням (в Омах)
// Для 100А струму на 0.008 Ом падіння складе ~0.8В. Додаємо до виміряної напруги під розрядом.
const float SYS_INTERNAL_RESISTANCE = 0.008f; 

// ========================= NIGHT MODE SETTINGS =========================
uint8_t nightStart = 23;  // 23:00
uint8_t nightEnd = 7;     // 07:00

// ========================= PINOUT (Wemos D1 mini) =========================
#define PIN_I2C_SCL D1
#define PIN_I2C_SDA D2
#define PIN_AC_DETECT D7  // LOW = AC present
#define PIN_BUZZER D0     // active buzzer 3V
#define PIN_ENC_A D6
#define PIN_ENC_B D5
#define PIN_ENC_BTN RX    // GPIO3

// ========================= LCD =========================
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t lcdPage = 0;
const uint8_t LCD_PAGES = 3;

// ========================= Battery =========================
enum BatteryType { LIFEPO4 = 0,
                   LEAD_ACID = 1,
                   GEL = 2 };
const char* batteryNames[] = { "LiFePo", "Acid", "GEL" };

// >>> default (LiFePO4 100Ah) <<<
BatteryType activeBattery = LIFEPO4;
BatteryType selectedBattery = LIFEPO4;

// ========================= INA226 =========================
static const uint8_t INA = 0x44;
static const uint8_t REG_CONFIG = 0x00;
static const uint8_t REG_SHUNT = 0x01;
static const uint8_t REG_BUS = 0x02;
static const uint8_t REG_POWER = 0x03;
static const uint8_t REG_CURR = 0x04;
static const uint8_t REG_CAL = 0x05;

static const float R_SHUNT = 0.000375f;  // 75mV/200A
static const float CURRENT_LSB = 0.01f;  // 10mA/LSB

// ================= Калібровка напруги (множник) ================
//     підірати по виміряній напрузі акумулятора після шунта
float VBUS_K = 0.945f; 

// ========================= Forward decl =========================
bool isNightNow();
String urlEnc(String s);

// ================================================================
long lastStatusRepliedUpdId = 0;

// ========================= INA helpers =========================
uint16_t r16(uint8_t reg) {
  Wire.beginTransmission(INA);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA, (uint8_t)2);
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}
void w16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(INA);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  Wire.endTransmission();
}
void inaInit() {
  w16(REG_CONFIG, 0x4527);
  uint16_t cal = (uint16_t)(0.00512f / (CURRENT_LSB * R_SHUNT) + 0.5f);
  w16(REG_CAL, cal);
}
bool inaRead(float& vBus, float& iA, float& pW, float& vSh_mV) {
  uint16_t rawBus = r16(REG_BUS);
  int16_t rawSh = (int16_t)r16(REG_SHUNT);
  int16_t rawCur = (int16_t)r16(REG_CURR);
  uint16_t rawP = r16(REG_POWER);

  vBus = rawBus * 0.00125f * VBUS_K;  // V
  vSh_mV = rawSh * 0.0025f;           // mV
  iA = rawCur * CURRENT_LSB;          // A signed
  pW = rawP * (25.0f * CURRENT_LSB);
  if (iA < 0) pW = -pW;

  if (vBus < 0.0f || vBus > 40.0f) return false;
  return true;
}

// Отримання "чистої" напруги акумулятора без урахування падіння під навантаженням (OCV)
float getEstimatedOCV(float vBus, float iBat) {
  // Якщо іде розряд (iBat > 0), напруга на клемах нижча за хімічну на величину падіння (I * R)
  // Якщо заряд (iBat < 0), напруга на клемах вища.
  return vBus + (iBat * SYS_INTERNAL_RESISTANCE);
}

// ========================= SOC tables =========================
struct BatteryLimits {
  float warnLow;
  float criticalLow;
};
BatteryLimits limits;

struct SocPoint {
  float voltage;
  int percent;
};

const SocPoint lifepo4Soc[] = {
  { 13.6, 100 }, { 13.4, 95 }, { 13.2, 90 }, { 13.1, 85 }, { 13.0, 80 }, { 12.9, 70 }, { 12.8, 60 }, { 12.7, 50 }, { 12.6, 40 }, { 12.5, 30 }, { 12.4, 20 }, { 12.2, 10 }, { 12.1, 5 }, { 12.0, 0 }
};
const int lifepo4SocSize = sizeof(lifepo4Soc) / sizeof(SocPoint);

const SocPoint leadSoc[] = {
  { 12.7, 100 }, { 12.6, 90 }, { 12.5, 80 }, { 12.4, 70 }, { 12.3, 60 }, { 12.2, 50 }, { 12.1, 40 }, { 12.0, 30 }, { 11.9, 20 }, { 11.8, 10 }, { 11.6, 0 }
};
const int leadSocSize = sizeof(leadSoc) / sizeof(SocPoint);

const SocPoint gelSoc[] = {
  { 12.85, 100 }, { 12.75, 90 }, { 12.65, 80 }, { 12.55, 70 }, { 12.45, 60 }, { 12.35, 50 }, { 12.25, 40 }, { 12.15, 30 }, { 12.05, 20 }, { 11.95, 10 }, { 11.8, 0 }
};
const int gelSocSize = sizeof(gelSoc) / sizeof(SocPoint);

void setBatteryLimits(BatteryType bt) {
  switch (bt) {
    case LIFEPO4:
      limits.warnLow = 12.4;
      limits.criticalLow = 12.0;
      break;
    case LEAD_ACID:
      limits.warnLow = 11.8;
      limits.criticalLow = 11.2;
      break;
    case GEL:
      limits.warnLow = 12.0;
      limits.criticalLow = 11.4;
      break;
  }
}

int calcSOC_byVoltage(float v) {
  const SocPoint* table = nullptr;
  int size = 0;

  switch (activeBattery) {
    case LIFEPO4:
      table = lifepo4Soc;
      size = lifepo4SocSize;
      break;
    case LEAD_ACID:
      table = leadSoc;
      size = leadSocSize;
      break;
    case GEL:
      table = gelSoc;
      size = gelSocSize;
      break;
  }

  for (int i = 0; i < size - 1; i++) {
    if (v >= table[i].voltage) {
      float v1 = table[i].voltage, v2 = table[i + 1].voltage;
      int p1 = table[i].percent, p2 = table[i + 1].percent;
      int out = (int)(p2 + (v - v2) * (p1 - p2) / (v1 - v2));
      if (out < 0) out = 0;
      if (out > 100) out = 100;
      return out;
    }
  }
  return 0;
}

// ====== CAPACITY (Повна ємність акумулятора за замовчуванням) ====== 
const float CAP_LFP_Wh = 1280.0f; // Wh
const float CAP_LEAD_Ah = 70.0f; // Ah
const float CAP_GEL_Ah = 100.0f; // Ah
const float LEAD_NOMINAL_V = 12.0f; // V

float capacityWh() {
  if (activeBattery == LIFEPO4) return CAP_LFP_Wh;
  if (activeBattery == LEAD_ACID) return CAP_LEAD_Ah * LEAD_NOMINAL_V;
  return CAP_GEL_Ah * LEAD_NOMINAL_V;
}

double storedWh = 0;

void initStoredFromVoltage(float vBat) {
  int soc = calcSOC_byVoltage(vBat);
  storedWh = capacityWh() * (soc / 100.0);
  if (storedWh < 0) storedWh = 0;
  if (storedWh > capacityWh()) storedWh = capacityWh();
}

int socFromStored() {
  float cap = capacityWh();
  if (cap < 1) return calcSOC_byVoltage(0);
  int s = (int)(100.0 * storedWh / cap + 0.5);
  if (s < 0) s = 0;
  if (s > 100) s = 100;
  return s;
}

// ========================= ALARM LOGIC =========================
#define WARN_HYST 0.10f
#define CRIT_HYST 0.20f
#define WARN_BEEP_INTERVAL 30000UL
#define CRIT_BEEP_INTERVAL 1000UL

bool warnActive = false, criticalActive = false;
bool warnMuted = false, criticalMuted = false;
uint32_t lastWarnBeep = 0, lastCritBeep = 0;

bool beepEnabled = true;
float MIN_LOAD_A_FOR_ALARM = 1.0f;

void buzzerOn() { digitalWrite(PIN_BUZZER, HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, LOW); }

void beepShort() {
  if (!beepEnabled || isNightNow()) return;
  buzzerOn();
  delay(80);
  buzzerOff();
}
void beepLong() {
  if (!beepEnabled || isNightNow()) return;
  buzzerOn();
  delay(350);
  buzzerOff();
}

// --- Non-blocking AC restored melody (4 long beeps) ---
bool acBeepActive = false;
bool acBeepOn = false;
uint8_t acBeepDone = 0;
uint32_t acBeepNextMs = 0;

const uint16_t AC_BEEP_ON_MS = 400;
const uint16_t AC_BEEP_OFF_MS = 120;

void startBeepACrestored() {
  if (!beepEnabled || isNightNow()) return;
  acBeepActive = true;
  acBeepOn = false;
  acBeepDone = 0;
  acBeepNextMs = millis();
}

void serviceBeepACrestored() {
  if (!acBeepActive) return;
  if (!beepEnabled || isNightNow()) {
    buzzerOff();
    acBeepActive = false;
    return;
  }
  uint32_t now = millis();
  if ((int32_t)(now - acBeepNextMs) < 0) return;

  if (!acBeepOn) {
    buzzerOn();
    acBeepOn = true;
    acBeepNextMs = now + AC_BEEP_ON_MS;
  } else {
    buzzerOff();
    acBeepOn = false;
    acBeepDone++;
    if (acBeepDone >= 4) acBeepActive = false;
    else acBeepNextMs = now + AC_BEEP_OFF_MS;
  }
}

void checkVoltageAlarms(float vClean, float iBat) {
  uint32_t now = millis();
  bool hasLoad = fabs(iBat) >= MIN_LOAD_A_FOR_ALARM;

  if (!warnActive && !criticalActive && hasLoad && vClean <= limits.warnLow && vClean > limits.criticalLow) {
    warnActive = true;
    warnMuted = false;
    lastWarnBeep = 0;
    beepShort();
  }
  if (warnActive && !warnMuted && (now - lastWarnBeep > WARN_BEEP_INTERVAL)) {
    beepShort();
    lastWarnBeep = now;
  }
  if (warnActive && vClean > limits.warnLow + WARN_HYST) {
    warnActive = false;
    warnMuted = false;
  }

  if (!criticalActive && hasLoad && vClean <= limits.criticalLow) {
    criticalActive = true;
    criticalMuted = false;
    lastCritBeep = 0;
    beepLong();
  }
  if (criticalActive && !criticalMuted && (now - lastCritBeep > CRIT_BEEP_INTERVAL)) {
    beepLong();
    lastCritBeep = now;
  }
  if (criticalActive && vClean > limits.criticalLow + CRIT_HYST) {
    criticalActive = false;
    criticalMuted = false;
  }
}

// ========================= SOC LOGIC =========================
const float I_REST_A = 0.25f;
const uint32_t REST_TIME_MS = 5UL * 60UL * 1000UL;
const float SOC_RESYNC_ALPHA = 0.12f;

const uint32_t FULL_DETECT_MS = 120UL * 1000UL;
const uint32_t EMPTY_DETECT_MS = 15UL * 1000UL;
const float FULL_TAIL_A = 1.5f;

float fullVoltageThreshold() {
  switch (activeBattery) {
    case LIFEPO4: return 14.2f;
    case LEAD_ACID: return 14.2f;
    case GEL: return 14.1f;
  }
  return 14.2f;
}

uint32_t restSinceMs = 0;
uint32_t fullSinceMs = 0;
uint32_t emptySinceMs = 0;

void socResyncAndEvents(float vClean, float iBat) {
  const float absI = fabs(iBat);

  // РЕСИНК дозволяємо ТІЛЬКИ в стані глибокого спокою (коли струм близький до нуля і немає заряду/розряду)
  // Забороняємо ресинк під час заряду (iBat < -0.05) та під час активного розряду, 
  // щоб скачки напруги від зарядки чи навантаження не ламали лічильник storedWh!
  bool isCharging = (iBat < -0.05f);
  
  if (!isCharging && absI <= I_REST_A) {
    if (restSinceMs == 0) restSinceMs = millis();
    if (millis() - restSinceMs >= REST_TIME_MS) {
      int socV = calcSOC_byVoltage(vClean);
      double targetWh = (double)capacityWh() * (socV / 100.0);
      storedWh = storedWh * (1.0 - SOC_RESYNC_ALPHA) + targetWh * SOC_RESYNC_ALPHA;
      double cap = capacityWh();
      if (storedWh < 0) storedWh = 0;
      if (storedWh > cap) storedWh = cap;
    }
  } else {
    restSinceMs = 0;
  }

  // FULL detect (залишається без змін: ловимо повний заряд чесно по досягненню порогової напруги і малому струму)
  bool fullCond = isCharging && (vClean >= fullVoltageThreshold()) && (fabs(iBat) <= FULL_TAIL_A);
  if (fullCond) {
    if (fullSinceMs == 0) fullSinceMs = millis();
    if (millis() - fullSinceMs >= FULL_DETECT_MS) storedWh = capacityWh();
  } else {
    fullSinceMs = 0;
  }

  // EMPTY detect
  bool discharging = (iBat > +0.2f);
  bool emptyCond = discharging && (vClean <= limits.criticalLow) && (absI >= MIN_LOAD_A_FOR_ALARM);
  if (emptyCond) {
    if (emptySinceMs == 0) emptySinceMs = millis();
    if (millis() - emptySinceMs >= EMPTY_DETECT_MS) storedWh = 0;
  } else {
    emptySinceMs = 0;
  }
}

// ========================= AC detect =========================
bool acStable = false, acPrev = false;
bool acRawLast = false;
uint32_t acFlipMs = 0;
const uint32_t AC_DEBOUNCE_MS = 700;
bool acReady = false;
uint32_t bootIgnoreUntilMs = 0;

bool readAcDebounced() {
  bool raw = (digitalRead(PIN_AC_DETECT) == LOW);
  if (raw != acRawLast) {
    acRawLast = raw;
    acFlipMs = millis();
  }
  if (millis() - acFlipMs >= AC_DEBOUNCE_MS) {
    acStable = raw;
  }
  return acStable;
}

// ========================= REAL TIME (NTP + TZ) =========================
bool timeValid = false;
time_t lastEpoch = 0;
uint32_t lastEpochMs = 0;

void timeSyncNTP() {
  configTime("EET-2EEST,M3.5.0/3,M10.5.0/4",
             "pool.ntp.org",
             "time.google.com",
             "time.windows.com");

  for (int i = 0; i < 40; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      timeValid = true;
      lastEpoch = now;
      lastEpochMs = millis();
      return;
    }
    delay(250);
  }
}

time_t nowEpoch() {
  if (!timeValid) return 0;
  uint32_t dt = millis() - lastEpochMs;
  return lastEpoch + (dt / 1000UL);
}

String fmtTimeLocal() {
  if (!timeValid) return String("no time");
  time_t e = nowEpoch();
  struct tm t;
  localtime_r(&e, &t);
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d-%02d %02d:%02d:%02d",
           t.tm_mday, t.tm_mon + 1, t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

// ========================= NIGHT MODE =========================
bool isNightNow() {
  if (!timeValid) return false;
  time_t e = nowEpoch();
  struct tm t;
  localtime_r(&e, &t);
  int h = t.tm_hour;
  if (nightStart < nightEnd) return (h >= nightStart && h < nightEnd);
  return (h >= nightStart || h < nightEnd);
}

// ========================= ENERGY / SESSION STATS =========================
double Wh_in_total = 0, Wh_out_total = 0;

// Сесійні змінні (один цикл: блекаут ↔ поява мережі)
double Wh_out_outage = 0;     // Витрачено за поточний/минулий блекаут
float peakW_outage = 0;       // Пікова потужність за блекаут
double Wh_in_acOn = 0;        // Залито енергії за період наявності мережі

uint32_t lastIntegrMs = 0;

void integrateEnergy(float vBat, float iBat) {
  uint32_t now = millis();
  if (lastIntegrMs == 0) {
    lastIntegrMs = now;
    return;
  }
  float dt_h = (now - lastIntegrMs) / 3600000.0f;
  lastIntegrMs = now;
  if (dt_h <= 0 || dt_h > 0.05f) return;

  if (iBat < 0) { // Заряд (струм мінусовий)
    double Wh = (double)(vBat * (-iBat) * dt_h);
    Wh_in_total += Wh;
    if (acStable) Wh_in_acOn += Wh;
    storedWh += Wh * ETA_CHARGE;
  } else { // Розряд (струм плюсовий)
    double Wh = (double)(vBat * (iBat) * dt_h);
    Wh_out_total += Wh;
    if (!acStable) {
      Wh_out_outage += Wh;
      float pW = vBat * iBat;
      if (pW > peakW_outage) peakW_outage = pW;
    }
    storedWh -= Wh / ETA_CHARGE;
  }

  float cap = capacityWh();
  if (storedWh < 0) storedWh = 0;
  if (storedWh > cap) storedWh = cap;
}

// ========================= WIFI + TELEGRAM =========================
uint32_t lastBotPollMs = 0;
long tgOffset = 0;

bool wifiOnAndConnect(uint32_t timeoutMs = 20000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(200);
  return WiFi.status() == WL_CONNECTED;
}
void wifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

String urlEnc(String s) {
  s.replace("%", "%25");
  s.replace(" ", "%20");
  s.replace("\n", "%0A");
  s.replace("\r", "");
  s.replace("&", "%26");
  s.replace("?", "%3F");
  s.replace("#", "%23");
  s.replace("\"", "%22");
  return s;
}

// ---- Telegram: sendMessage ----
bool tgSendTo(const String& chatId, const String& text) {
  WiFiClientSecure c;
  c.setInsecure();
  c.setTimeout(6000);

  if (!c.connect("api.telegram.org", 443)) return false;

  String url = "/bot" + String(TG_BOT_TOKEN) + "/sendMessage?chat_id=" + chatId + "&text=" + urlEnc(text);

  c.print(String("GET ") + url + " HTTP/1.1\r\n"
                                 "Host: api.telegram.org\r\n"
                                 "Connection: close\r\n\r\n");

  String statusLine = c.readStringUntil('\n');
  statusLine.trim();
  bool httpOk = (statusLine.indexOf("200") >= 0);

  while (c.connected()) {
    String line = c.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  String body;
  uint32_t t0 = millis();
  while (c.connected() && millis() - t0 < 3500) {
    while (c.available()) {
      char ch = (char)c.read();
      if (body.length() < 2048) body += ch;
    }
    if (body.indexOf("\"ok\":true") >= 0) break;
    delay(5);
  }
  c.stop();

  return httpOk && (body.indexOf("\"ok\":true") >= 0);
}

// ---- Telegram: getUpdates -> last /status ----
bool tgPollStatusCommand(String& replyChatId) {
  replyChatId = "";

  WiFiClientSecure c;
  c.setInsecure();
  const int LIMIT = 5;
  const int TIMEOUT = 5;
  c.setTimeout((TIMEOUT + 5) * 1000);

  if (!c.connect("api.telegram.org", 443)) return false;

  String url = "/bot" + String(TG_BOT_TOKEN) + "/getUpdates?timeout=" + String(TIMEOUT) + "&limit=" + String(LIMIT) + "&offset=" + String(tgOffset) + "&allowed_updates=%5B%22message%22%5D";

  c.print(String("GET ") + url + " HTTP/1.1\r\n"
                                 "Host: api.telegram.org\r\n"
                                 "Connection: close\r\n\r\n");

  while (c.connected()) {
    String line = c.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  StaticJsonDocument<512> filter;
  filter["ok"] = true;

  for (int i = 0; i < LIMIT; i++) {
    JsonObject fi = filter["result"][i].to<JsonObject>();
    fi["update_id"] = true;

    JsonObject fm = fi["message"].to<JsonObject>();
    fm["text"] = true;
    fm["date"] = true;
    fm["chat"]["id"] = true;
  }

  DynamicJsonDocument doc(6144);
  DeserializationError err = deserializeJson(doc, c, DeserializationOption::Filter(filter));
  c.stop();
  if (err) return false;

  if (!doc["ok"].is<bool>() || !doc["ok"].as<bool>()) return false;

  JsonArray res = doc["result"].as<JsonArray>();
  if (res.isNull() || res.size() == 0) return false;

  long maxUpdId = 0;
  bool foundStatus = false;
  long bestUpdId = 0;
  long bestChatId = 0;
  long bestMsgDate = 0;

  for (JsonObject upd : res) {
    long updId = upd["update_id"] | 0;
    if (updId > maxUpdId) maxUpdId = updId;

    JsonObject msg = upd["message"];
    if (msg.isNull()) continue;

    const char* text = msg["text"] | "";
    if (!text[0]) continue;

    if (strncmp(text, "/status", 7) != 0) continue;

    long chatId = msg["chat"]["id"] | 0;
    long msgDate = msg["date"] | 0;

    if (!foundStatus || updId >= bestUpdId) {
      foundStatus = true;
      bestUpdId = updId;
      bestChatId = chatId;
      bestMsgDate = msgDate;
    }
  }

  if (maxUpdId > 0) tgOffset = maxUpdId + 1;
  else return false;

  if (!foundStatus) return false;

  if (bestUpdId <= lastStatusRepliedUpdId) return false;

  if (timeValid && bestMsgDate > 0) {
    long age = (long)nowEpoch() - bestMsgDate;
    if (age > 600) return false; // ВИПРАВЛЕНО: 10 хвилин (600 секунд)
  }

  lastStatusRepliedUpdId = bestUpdId;
  replyChatId = String(bestChatId);
  return true;
}

static long findMaxUpdateIdInBody(const String& body) {
  long maxId = 0;
  int pos = 0;
  while (true) {
    int p = body.indexOf("\"update_id\":", pos);
    if (p < 0) break;
    p += 12;
    while (p < (int)body.length() && (body[p] == ' ')) p++;
    long v = 0;
    bool any = false;
    while (p < (int)body.length()) {
      char ch = body[p];
      if (ch < '0' || ch > '9') break;
      any = true;
      v = v * 10 + (ch - '0');
      p++;
    }
    if (any && v > maxId) maxId = v;
    pos = p;
  }
  return maxId;
}

void tgInitOffsetCatchUp() {
  WiFiClientSecure c;
  c.setInsecure();
  c.setTimeout(7000);
  if (!c.connect("api.telegram.org", 443)) return;

  const int LIMIT = 50;
  String url = "/bot" + String(TG_BOT_TOKEN) + "/getUpdates?timeout=0&limit=" + String(LIMIT) + "&offset=" + String(tgOffset);

  c.print(String("GET ") + url + " HTTP/1.1\r\n"
                                 "Host: api.telegram.org\r\n"
                                 "Connection: close\r\n\r\n");

  while (c.connected()) {
    String line = c.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  String body;
  uint32_t t0 = millis();
  while (c.connected() && millis() - t0 < 4000) {
    while (c.available()) {
      body += (char)c.read();
      if (body.length() > 30000) body.remove(0, 10000);
    }
    delay(5);
  }
  c.stop();

  long maxUpdId = findMaxUpdateIdInBody(body);
  if (maxUpdId > 0) {
    tgOffset = maxUpdId + 1;
    lastStatusRepliedUpdId = maxUpdId;
  }
}

// ========================= UI (encoder) =========================
enum UiMode { MODE_MONITOR, MODE_SELECT_BATTERY };
UiMode uiMode = MODE_MONITOR;

uint32_t btnPressTime = 0;
bool btnPrev = HIGH;
bool shortPressFlag = false;
bool longPressFlag = false;

void updateButton() {
  static bool longHandled = false;
  bool btn = digitalRead(PIN_ENC_BTN);

  if (btnPrev == HIGH && btn == LOW) {
    btnPressTime = millis();
    longHandled = false;
  }
  if (btnPrev == LOW && btn == HIGH) {
    if (!longHandled && millis() - btnPressTime < 500) shortPressFlag = true;
  }
  if (btn == LOW && millis() - btnPressTime > 1200 && !longHandled) {
    longPressFlag = true;
    longHandled = true;
  }
  btnPrev = btn;
}

int8_t readEncoderStep() {
  static uint8_t lastAB = 0;
  static int8_t acc = 0;
  static uint32_t lastMs = 0;

  uint32_t now = millis();
  if (now - lastMs < 3) return 0;
  lastMs = now;

  uint8_t a = digitalRead(PIN_ENC_A) ? 1 : 0;
  uint8_t b = digitalRead(PIN_ENC_B) ? 1 : 0;
  uint8_t ab = (a << 1) | b;

  static const int8_t tbl[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
  };

  int8_t d = tbl[(lastAB << 2) | ab];
  lastAB = ab;
  if (d == 0) return 0;

  acc += d;
  if (acc >= 4) {
    acc = 0;
    return +1;
  }
  if (acc <= -4) {
    acc = 0;
    return -1;
  }
  return 0;
}

void updateEncoderSelect() {
  int8_t s = readEncoderStep();
  if (!s) return;

  int b = (int)selectedBattery + (s > 0 ? 1 : -1);
  if (b < (int)LIFEPO4) b = (int)GEL;
  if (b > (int)GEL) b = (int)LIFEPO4;
  selectedBattery = (BatteryType)b;
  beepShort();
}

// ========================= LCD =========================
float vBat = 0, iBat = 0, pBat = 0, vSh = 0;

void print16(const char* s) {
  char out[17];
  snprintf(out, sizeof(out), "%-16.16s", s);
  lcd.print(out);
}

void drawMonitor() {
  char top[17];
  snprintf(top, sizeof(top), "%s %s %s",
           batteryNames[activeBattery],
           acStable ? "AC:ON" : "AC:OFF",
           (warnMuted || criticalMuted) ? "M+" : "M-");
  lcd.setCursor(0, 0);
  print16(top);

  lcd.setCursor(0, 1);
  char buf[32];

  if (lcdPage == 0) {
    int socS = socFromStored();
    float pW_ = vBat * iBat;
    snprintf(buf, sizeof(buf), "%4.1fV %4.0fW %3d%%", vBat, pW_, socS);
    print16(buf);
  } else if (lcdPage == 1) {
    // Сторінка 1 показує сесійні дані залежно від стану мережі
    if (!acStable) {
      snprintf(buf, sizeof(buf), "Out%4.0fWh Pk%4.0f", (float)Wh_out_outage, peakW_outage);
    } else {
      snprintf(buf, sizeof(buf), "In:%4.0fWh AC ON", (float)Wh_in_acOn);
    }
    print16(buf);
  } else {
    snprintf(buf, sizeof(buf), "Tot_In:%4.0fWh", (float)Wh_in_total);
    print16(buf);
  }
}

void drawSelect() {
  lcd.setCursor(0, 0);
  print16("Select battery ");
  lcd.setCursor(0, 1);
  char line[17];
  snprintf(line, sizeof(line), "> %-13s", batteryNames[selectedBattery]);
  print16(line);
}

// ========================= AC events =========================
bool pendingRestore = false;
uint32_t restoreSeenMs = 0;

bool pendingLost = false;
uint32_t lostSeenMs = 0;

void sendAcLost() {
  String msg;
  msg = "💩 рососія підараси\n";
  msg += "❌ Мережа зникла\n";
  msg += "🕒 " + fmtTimeLocal() + "\n";
  msg += "🔋 U=" + String(vBat, 2) + "V\n";
  msg += "📥 Wh_in(AC on)=" + String((float)Wh_in_acOn, 1) + "\n";
  msg += "⚡ SOC=" + String(socFromStored()) + "% (" + String((float)storedWh, 0) + "Wh)";

  tgSendTo(String(TG_CHAT_ID), msg);

  // Скидаємо сесійний лічильник заряду, починаємо відлік блекауту
  Wh_in_acOn = 0;
  Wh_out_outage = 0;
  peakW_outage = 0;
}

void sendAcRestored() {
  String msg;
  msg = "🫡 Енергетики найкращі\n";
  msg += "✅ Мережа з'явилась\n";
  msg += "🕒 " + fmtTimeLocal() + "\n";
  msg += "🔋 U=" + String(vBat, 2) + "V\n";
  msg += "📤 Wh_used(outage)=" + String((float)Wh_out_outage, 1) + "\n";
  msg += "📈 PeakW(outage)=" + String(peakW_outage, 0) + "\n";
  msg += "⚡ SOC=" + String(socFromStored()) + "% (" + String((float)storedWh, 0) + "Wh)";

  tgSendTo(String(TG_CHAT_ID), msg);

  // Скидаємо сесійний лічильник витрат блекауту
  Wh_out_outage = 0;
  peakW_outage = 0;
  Wh_in_acOn = 0;
}

// ========================= SETUP / LOOP =========================
void setup() {
  pinMode(PIN_AC_DETECT, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Boot...");

  inaInit();
  setBatteryLimits(activeBattery);

  warnActive = criticalActive = false;
  warnMuted = criticalMuted = false;
  lastWarnBeep = lastCritBeep = 0;
  beepEnabled = true;

  acRawLast = (digitalRead(PIN_AC_DETECT) == LOW);
  acStable = acRawLast;
  acPrev = acStable;
  acFlipMs = millis();
  pendingRestore = false;
  acReady = false;
  bootIgnoreUntilMs = millis() + 2000;

  // Boot: WiFi ON once -> NTP -> catch up offset -> OFF
  if (wifiOnAndConnect(20000)) {
    timeSyncNTP();
    tgInitOffsetCatchUp();
  }
  wifiOff();

  // First voltage -> init storedWh
  float vb, ia, pw, sh;
  for (int i = 0; i < 3; i++) {
    if (inaRead(vb, ia, pw, sh)) {
      vBat = vb;
      iBat = ia;
      pBat = pw;
      vSh = sh;
    }
    delay(200);
  }
  initStoredFromVoltage(vBat);
  restSinceMs = fullSinceMs = emptySinceMs = 0;

  lastIntegrMs = millis();
  lastBotPollMs = millis();

  lcd.clear();
  acReady = true;
}

void loop() {
  // INA read
  static uint32_t lastInaMs = 0;
  if (millis() - lastInaMs > 500) {
    float vb, ia, pw, sh;
    if (inaRead(vb, ia, pw, sh)) {
      const float a = 0.2f;
      vBat = (vBat == 0) ? vb : (vBat * (1.0f - a) + vb * a); // ВИПРАВЛЕНО: 1.0f - a
      iBat = iBat * (1.0f - a) + ia * a;                     // ВИПРАВЛЕНО: 1.0f - a
      pBat = pBat * (1.0f - a) + pw * a;                     // ВИПРАВЛЕНО: 1.0f - a
      vSh = sh;

      // Вираховуємо "чисту" напругу без урахування падіння під навантаженням
      float vClean = getEstimatedOCV(vBat, iBat);

      integrateEnergy(vBat, iBat);
      socResyncAndEvents(vClean, iBat);     // Передаємо чисту напругу для перевірки SOC та порогів
      checkVoltageAlarms(vClean, iBat);     // Перевіряємо тривоги по чистій напрузі
    }
    lastInaMs = millis();
  }

  // AC state machine
  readAcDebounced();
  bool allowAcEvents = acReady && (millis() >= bootIgnoreUntilMs);

  if (allowAcEvents && (acStable != acPrev)) {
    acPrev = acStable;
    if (!acStable) {
      beepShort();
      pendingLost = true;
      lostSeenMs = millis();
      pendingRestore = false;
    } else {
      startBeepACrestored();
      pendingRestore = true;
      restoreSeenMs = millis();
      pendingLost = false;
    }
  } else {
    if (!allowAcEvents) acPrev = acStable;
  }

  // delayed LOST notify
  if (pendingLost) {
    if (!acStable) {
      if (millis() - lostSeenMs >= AC_NOTIFY_DELAY_MS) {
        if (wifiOnAndConnect(20000)) {
          timeSyncNTP();
          sendAcLost();
        }
        wifiOff();
        pendingLost = false;
      }
    } else pendingLost = false;
  }

  // delayed RESTORE notify
  if (pendingRestore) {
    if (acStable) {
      if (millis() - restoreSeenMs >= AC_NOTIFY_DELAY_MS) {
        if (wifiOnAndConnect(20000)) {
          timeSyncNTP();
          sendAcRestored();
        }
        wifiOff();
        pendingRestore = false;
      }
    } else pendingRestore = false;
  }

  // periodic bot poll (/status)
  const uint32_t pollEveryMs = BOT_POLL_EVERY_MIN * 60UL * 1000UL;

  if (millis() - lastBotPollMs >= pollEveryMs) {
    lastBotPollMs = millis();

    if (wifiOnAndConnect(15000)) {
      WiFi.setSleepMode(WIFI_NONE_SLEEP);

      delay(150);
      timeSyncNTP();

      String replyId;
      if (tgPollStatusCommand(replyId)) {
        float pW_ = vBat * iBat;

        String msg;
        msg = "📟 STATUS\n";
        msg += "🕒 " + fmtTimeLocal() + "\n";
        msg += String("AC: ") + (acStable ? "ON\n" : "OFF\n");
        msg += "BAT: " + String(batteryNames[activeBattery]) + "\n";
        msg += "U=" + String(vBat, 2) + "V I=" + String(iBat, 2) + "A\n";
        msg += "P=" + String(pW_, 0) + "W\n";
        msg += "SOC=" + String(socFromStored()) + "% (" + String((float)storedWh, 0) + "Wh)\n";
        msg += "Wh_in=" + String((float)Wh_in_total, 1) + " Wh_out=" + String((float)Wh_out_total, 1);

        tgSendTo(replyId, msg);
      }
    }

    wifiOff();
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
  }

  // UI
  updateButton();

  if (uiMode == MODE_MONITOR) {
    int8_t step = readEncoderStep();
    if (step) {
      if (step > 0) lcdPage = (lcdPage + 1) % LCD_PAGES;
      else lcdPage = (lcdPage + LCD_PAGES - 1) % LCD_PAGES;
      beepShort();
    }

    if (shortPressFlag) {
      if (criticalActive) criticalMuted = true;
      else if (warnActive) warnMuted = true;
      beepShort();
    }

    if (longPressFlag) {
      uiMode = MODE_SELECT_BATTERY;
      selectedBattery = activeBattery;
      lcd.clear();
      beepLong();
    }
  } else {
    updateEncoderSelect();

    if (longPressFlag) {
      activeBattery = selectedBattery;
      setBatteryLimits(activeBattery);
      initStoredFromVoltage(vBat);

      restSinceMs = fullSinceMs = emptySinceMs = 0;
      warnActive = criticalActive = false;
      warnMuted = criticalMuted = false;
      lastWarnBeep = lastCritBeep = 0;

      uiMode = MODE_MONITOR;
      lcd.clear();
      beepLong();
    }
  }

  shortPressFlag = false;
  longPressFlag = false;

  // LCD
  static uint32_t lastLcdMs = 0;
  if (millis() - lastLcdMs > 500) {
    if (uiMode == MODE_MONITOR) drawMonitor();
    else drawSelect();
    lastLcdMs = millis();
  }

  serviceBeepACrestored();
  delay(5);
}
