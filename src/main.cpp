#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <time.h>
#include <vector>
#include <math.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_partition.h>
#include "shaon_fonts.h"

Preferences prefs;

// ---- Config ----
// WiFi credentials, zip code, and location are runtime-mutable, loaded
// from Preferences (NVS) on fresh boot, with these hardcoded values
// used only as the very first fallback default before any setup has
// been done via the WiFi/Location Setup captive portal (see
// startProvisioning()).
//
// BUG FIX 2026-07-13: these five variables were previously PLAIN
// globals (not RTC_DATA_ATTR). On the ESP32, only RTC_DATA_ATTR
// variables survive deep sleep -- every other global resets to its
// compile-time initializer on each wake, since deep sleep is
// effectively a soft-reboot of the data segment. Since NVS is only
// re-read into these variables inside the `if (freshBoot)` block in
// setup(), every ordinary TIMER wake (the vast majority of wakes, once
// per minute) was silently resetting all five of these back to their
// hardcoded defaults ("NETGEAR75", "blackpotato833", empty zip, Palo
// Alto lat/lon) immediately after the fresh-boot wake that actually
// loaded the real configured values. This explains two real, confirmed
// symptoms: (1) the Settings screen showing zip "not set" even though a
// real zip (94301) was confirmed working in that session's fresh-boot
// serial log -- the in-memory value had already been wiped by the very
// next timer-wake; and (2) a more serious latent bug where
// wifiConnect(), which runs on nearly every wake (not just fresh boot),
// would have silently reconnected using the HARDCODED fallback WiFi
// credentials instead of whatever was actually configured via the
// captive portal, on every wake after the first. This had gone
// unnoticed only because testing happened to use a network matching
// the hardcoded default. Fixed by making all five RTC_DATA_ATTR, so
// they persist correctly across deep sleep between the initial
// fresh-boot NVS load and every subsequent wake, exactly like every
// other piece of cached state in this file (sunriseEpoch, heDay,
// currentScreen, etc).
//
// Buffer sizes: 33 bytes for SSID (max 32 chars + null terminator, per
// the 802.11 SSID length limit), 64 for password (max WPA2 passphrase is
// 63 chars + null), 8 for a 5-digit US zip + null with a little slack.
RTC_DATA_ATTR char wifiSsid[33] = "NETGEAR75";
RTC_DATA_ATTR char wifiPassword[64] = "blackpotato833";
RTC_DATA_ATTR char zipCode[8] = "";   // empty = not yet configured, falls back to GEONAMEID
RTC_DATA_ATTR float currentLatitude = 37.44;
RTC_DATA_ATTR float currentLongitude = -122.14;
const char* TZID = "America/Los_Angeles";
// NTP: Pacific time. TODO: proper DST/timezone handling later.
const long GMT_OFFSET_SEC = -8 * 3600;
const int  DST_OFFSET_SEC = 3600;

#define WAKE_INTERVAL_SECONDS 60
// ---- Button pins and roles (REMAPPED 2026-07-13) ----
// Physical corner positions, per the original hardware layout:
//   MENU_BTN_PIN (7) = bottom-left
//   BACK_BTN_PIN (6) = top-left
//   UP_BTN_PIN   (0) = top-right
//   DOWN_BTN_PIN (8) = bottom-right
// Roles as of this remapping (see the wake-handling block in setup()
// for the actual logic):
//   Top-right (UP_BTN_PIN)    -> cycle FORWARD through screens (was
//                                 previously MENU's job before the swap)
//   Top-left  (BACK_BTN_PIN)  -> cycle BACKWARD through screens
//                                 (unchanged)
//   Bottom-left (MENU_BTN_PIN)-> TAP: force a screen refresh. HOLD
//                                 (>=600ms): toggle Hebrew/English
//                                 globally, from any screen (was
//                                 previously UP's refresh-only job
//                                 before the swap; the hold-to-toggle
//                                 behavior is new)
//   Bottom-right (DOWN_BTN_PIN)-> Settings-only: TAP moves the row
//                                 cursor forward, HOLD toggles the
//                                 selected row (unchanged)
#define MENU_BTN_PIN 7
#define BACK_BTN_PIN 6
#define UP_BTN_PIN   0
#define DOWN_BTN_PIN 8
#define BTN_PIN_MASK ((1ULL<<MENU_BTN_PIN)|(1ULL<<BACK_BTN_PIN)|(1ULL<<UP_BTN_PIN)|(1ULL<<DOWN_BTN_PIN))

// ---- DEBUG: force provisioning on fresh boot (TEST COMPLETE, DISABLED) ----
#define DEBUG_FORCE_PROVISION_ON_BOOT false

// ---- WiFi / Location setup (captive portal) ----
#define SETUP_AP_SSID "Shaon-Setup"
IPAddress SETUP_AP_IP(192, 168, 4, 1);

// ---- Screen navigation ----
#define NUM_SCREENS 7
RTC_DATA_ATTR int currentScreen = 0;   // survives deep sleep, resets on power loss

// ---- Display / pins ----
#define DISPLAY_CS 33
#define DISPLAY_DC 34
#define DISPLAY_RES 35
#define DISPLAY_BUSY 36
#define SPI_MOSI 48
#define SPI_MISO 46
#define SPI_SCK 47

GxEPD2_BW<GxEPD2_154_GDEY0154D67, GxEPD2_154_GDEY0154D67::HEIGHT> display(
  GxEPD2_154_GDEY0154D67(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY));
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

// ---- Cached data that survives deep sleep ----
RTC_DATA_ATTR bool   haveData = false;
// ---- NEW 2026-07-13: fresh-boot data-fetch retry with backoff ----
// If the very first fresh-boot WiFi connect/NTP-sync/zmanim-fetch
// sequence fails (confirmed real scenario from a live serial log: "WiFi
// failed on fresh boot", leaving haveData=false and every data-
// dependent screen blank until the NEXT sunset rollover, potentially
// many hours later), this counter drives a retry attempt on ordinary
// TIMER wakes -- not just at the next rollover. Persists across deep
// sleep (RTC_DATA_ATTR) since the retry needs to span multiple wake
// cycles. Reset to 0 whenever a fetch actually succeeds (haveData
// becomes true), so a LATER failure starts the fast-retry phase fresh
// rather than staying in slow-backoff mode forever. See the retry logic
// itself (in setup(), guarded by `if (!haveData)`) for the actual
// fast/slow backoff schedule.
RTC_DATA_ATTR int    dataFetchRetryCount = 0;
RTC_DATA_ATTR time_t sunriseEpoch = 0;
RTC_DATA_ATTR time_t sunsetEpoch = 0;
RTC_DATA_ATTR time_t alotEpoch = 0;          // עלות השחר — dawn
RTC_DATA_ATTR time_t sofZmanShmaEpoch = 0;   // סוף זמן ק"ש — latest Shema (GRA)
RTC_DATA_ATTR time_t sofZmanShmaMGAEpoch = 0;// latest Shema (MGA) — Settings toggle picks between the two
RTC_DATA_ATTR time_t chatzotEpoch = 0;       // חצות — midday
RTC_DATA_ATTR time_t minchaGedolaEpoch = 0;  // מנחה גדולה — earliest Mincha
RTC_DATA_ATTR time_t tzeitEpoch = 0;         // צאת הכוכבים — nightfall (3 small stars, 8.5°)
RTC_DATA_ATTR int    omerDay = 0;            // 0 = not currently in the Omer count
RTC_DATA_ATTR char   heDay[24]   = "";
RTC_DATA_ATTR char   heMonth[24] = "";
RTC_DATA_ATTR char   heYear[24]  = "";
// ---- NEW: numeric Hebrew date fields for English-mode display ----
// heDay/heMonth/heYear above are gematria strings straight from Hebcal
// (e.g. "כ״ה"), which cannot be parsed back into a number for Arabic-
// numeral display. Hebcal's converter endpoint also returns plain
// numeric hy/hd fields and an English month name (hm) alongside
// heDateParts -- confirmed via real Hebcal API documentation/examples
// (e.g. {"hy":5771,"hm":"Iyyar","hd":29,...}). These three fields are
// fetched and stored separately specifically for English-mode display,
// so that corner never falls back to showing raw gematria out of
// context (previously: this corner was Hebrew-only regardless of the
// language setting, a real bug, not just a missing translation).
RTC_DATA_ATTR int    heDayNum = 0;
RTC_DATA_ATTR int    heYearNum = 0;
RTC_DATA_ATTR char   heMonthEn[16] = "";
RTC_DATA_ATTR float  moonFrac = 0.0;
RTC_DATA_ATTR time_t lastSyncEpoch = 0;   // when data was last successfully fetched
// ---- NEW: last successful full data sync, for the Settings "Last
// Updated" line (2026-07-13). Distinct from lastSyncEpoch in name only
// for clarity at the call site -- both are set at the same moments
// (fresh boot NTP sync, and successful sunset rollover). Kept as a
// separate variable rather than reusing lastSyncEpoch directly so the
// Settings display logic reads clearly as "what it shows", independent
// of lastSyncEpoch's other use in the syncFresh staleness check.
RTC_DATA_ATTR time_t lastSuccessfulUpdateEpoch = 0;
RTC_DATA_ATTR int    wakesSinceFullRefresh = 0;  // for periodic full-refresh to clear ghosting

// ---- Screen 4: next-holiday countdown (used when omerDay == 0) ----
RTC_DATA_ATTR char   nextHolidayHebrew[48] = "";
// ---- NEW: English holiday name, straight from Hebcal's own "title"
// field on the same holiday item as "hebrew" -- e.g. {"title":"Shavuot
// II","hebrew":"שבועות ב׳",...}. Confirmed via real Hebcal API
// documentation that holiday items always carry both fields together,
// so no separate lookup table or extra fetch is needed -- this was
// simpler than the originally-considered Hebrew-name-to-English lookup
// table approach, which would have needed ongoing maintenance to cover
// every holiday variant Hebcal can return.
RTC_DATA_ATTR char   nextHolidayEn[48] = "";
RTC_DATA_ATTR int    nextHolidayY = 0, nextHolidayM = 0, nextHolidayD = 0; // Gregorian date, midnight UTC basis
RTC_DATA_ATTR bool   haveNextHoliday = false;
RTC_DATA_ATTR time_t nextCandleEpoch = 0;   // true UTC — next candle-lighting moment
RTC_DATA_ATTR bool   haveNextCandle = false;
RTC_DATA_ATTR char   nextCandleMemo[48] = "";

// ---- Screen 4 (Torah Quotes): this week's parasha ----
// Fetched alongside the holiday/candle lookup in fetchNextHoliday(),
// reusing the same network round-trip and 120-day query window rather
// than a separate fetch. Stores whatever bare name Hebcal's own "title"
// field returns for the category=="parashat" item (e.g. "Bereshit",
// "Vayakhel-Pekudei"), with the "Parashat " prefix stripped -- this is
// deliberately NOT split/recombined in firmware; the CSV data file is
// expected to carry row-sets for every combined AND split variant a
// given parsha pair can take across different Hebrew years, so whatever
// string Hebcal returns is used as the lookup key verbatim.
RTC_DATA_ATTR char   torahParashaName[32] = "";
RTC_DATA_ATTR bool   haveTorahParasha = false;

// ---- Screen 5: weather (Open-Meteo) ----
RTC_DATA_ATTR float  weatherTemp = 0;
RTC_DATA_ATTR int    weatherCode = -1;       // WMO weather code, -1 = no data yet
RTC_DATA_ATTR int    weatherHumidity = 0;
RTC_DATA_ATTR float  weatherWindSpeed = 0;
RTC_DATA_ATTR int    weatherWindDir = 0;     // degrees, 0-359
RTC_DATA_ATTR bool   haveWeather = false;
RTC_DATA_ATTR int    wakesSinceWeatherFetch = 999; // force a fetch on first real wake
#define FORECAST_DAYS 6
RTC_DATA_ATTR int forecastCode[FORECAST_DAYS];
RTC_DATA_ATTR int forecastTempMax[FORECAST_DAYS];
RTC_DATA_ATTR int forecastWeekday[FORECAST_DAYS]; // 0=Sunday .. 6=Saturday

// ---- Settings (persisted to NVS via Preferences, mirrored here for fast
// access across deep-sleep wakes without a flash read every minute) ----
RTC_DATA_ATTR bool zmanimMethodMGA = false;   // false = GRA (default), true = MGA
RTC_DATA_ATTR bool tempFahrenheit = true;     // false = Celsius
RTC_DATA_ATTR bool languageEnglish = false;   // false = Hebrew/gematria (default), true = English/Arabic numerals
RTC_DATA_ATTR bool screenEnabled[6] = {true, true, true, true, true, true}; // indices 0-5 (Settings itself, index 6, is always on)
RTC_DATA_ATTR int  settingsCursor = 0;        // 0=language, 1=temp unit, 2=GRA/MGA, 3-8=screenEnabled[0..5], 9=WiFi Setup, 10=Zip (NEW 2026-07-13: own real cursor stop, was previously unaddressable), 11=Serve Debug Log

// ---- Battery / charge pins (Watchy v3) ----
#define BATT_ADC_PIN 9
#define CHRG_STATUS_PIN 10
#define USB_DET_PIN 21   // official Watchy v3 pin for USB-plugged-in detection
#define SYNC_STALE_SECONDS (25 * 3600)     // 25h without a sync => stale

RTC_DATA_ATTR time_t lastUsbConnectedEpoch = 0;   // last wake where USB was present
RTC_DATA_ATTR bool   wasUsbConnected = false;     // USB state as of the previous wake
RTC_DATA_ATTR bool   haveUsbBaseline = false;     // true once we've seen a real unplug transition
#define ASSUMED_BATTERY_LIFE_SECONDS (60L * 60 * 24 * 3)  // 3 days, WiFi-fetching usage

bool isDaytime = true;

// ---- Persistent wake log (2026-07-12) ----
#define LOG_FILE_PATH "/shaon_wake_log.txt"
#define LOG_MAX_BYTES (200 * 1024)
bool littleFsReady = false;

void ensureLittleFsMounted() {
  if (littleFsReady) return;
  littleFsReady = LittleFS.begin(true);
  if (!littleFsReady) {
    Serial.println("ensureLittleFsMounted: LittleFS mount+format failed -- logging disabled this session");
  }
}

void wakeLogAppend(const String& line) {
  ensureLittleFsMounted();
  if (!littleFsReady) return;
  File f = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("wakeLogAppend: failed to open log file for append");
    return;
  }
  f.println(line);
  f.close();

  File check = LittleFS.open(LOG_FILE_PATH, FILE_READ);
  if (check) {
    size_t sz = check.size();
    if (sz > LOG_MAX_BYTES) {
      String all; all.reserve(sz);
      while (check.available()) all += (char)check.read();
      check.close();
      int cut = all.length() / 2;
      while (cut < (int)all.length() && all[cut] != '\n') cut++;
      String kept = all.substring(cut + 1);
      File rewrite = LittleFS.open(LOG_FILE_PATH, FILE_WRITE); // truncates
      if (rewrite) { rewrite.print(kept); rewrite.close(); }
      Serial.printf("wakeLogAppend: log exceeded %d bytes, trimmed to %d bytes\n",
        LOG_MAX_BYTES, (int)kept.length());
    } else {
      check.close();
    }
  }
}

// ---- Torah Quotes CSV cache (2026-07-13) ----
// Fetched from a stable, documented GitHub raw-content URL rather than
// a live Google Sheets export -- avoids depending on Google's
// undocumented export-endpoint behavior (redirect-following, auth
// requirements that could change) and avoids writing a new CSV-parsing
// layer against an unstable source. Andrew edits the source Google
// Sheet, periodically exports to CSV, and commits it into the repo at
// data/torah_quotes.csv; this fetch just pulls that committed file.
#define TORAH_CSV_PATH "/torah_quotes.csv"
#define TORAH_CSV_URL "https://raw.githubusercontent.com/asam26/shaon/main/data/torah_quotes.csv"

// ---- Hebrew RTL helpers ----
String reverseUTF8(const char* str) {
  std::vector<String> chars; int i = 0;
  while (str[i] != '\0') {
    unsigned char c = str[i]; int len = 1;
    if ((c&0x80)==0x00) len=1; else if ((c&0xE0)==0xC0) len=2;
    else if ((c&0xF0)==0xE0) len=3; else if ((c&0xF8)==0xF0) len=4;
    chars.push_back(String(str).substring(i, i+len)); i += len;
  }
  String out=""; for (int j=chars.size()-1;j>=0;j--) out+=chars[j];
  return out;
}
void printHebrewCentered(const char* s, int cx, int y) {
  String r = languageEnglish ? String(s) : reverseUTF8(s);
  int w=u8g2Fonts.getUTF8Width(r.c_str());
  u8g2Fonts.setCursor(cx-w/2, y); u8g2Fonts.print(r);
}
void printHebrewRight(const char* s, int rx, int y) {
  String r = languageEnglish ? String(s) : reverseUTF8(s);
  int w=u8g2Fonts.getUTF8Width(r.c_str());
  u8g2Fonts.setCursor(rx-w, y); u8g2Fonts.print(r);
}
void printHebrewLeft(const char* s, int lx, int y) {
  String r = languageEnglish ? String(s) : reverseUTF8(s);
  u8g2Fonts.setCursor(lx, y); u8g2Fonts.print(r);
}
void printPlainCentered(const char* s, int cx, int y) {
  int w = u8g2Fonts.getUTF8Width(s);
  u8g2Fonts.setCursor(cx - w/2, y); u8g2Fonts.print(s);
}

void printBilingualCentered(const char* heText, const char* enText, int cx, int y,
                             const uint8_t* heFont, const uint8_t* enFont) {
  if (languageEnglish) {
    u8g2Fonts.setFont(enFont);
    printPlainCentered(enText, cx, y);
  } else {
    u8g2Fonts.setFont(heFont);
    printHebrewCentered(heText, cx, y);
  }
}
void printBilingualRight(const char* heText, const char* enText, int rx, int y,
                          const uint8_t* heFont, const uint8_t* enFont) {
  if (languageEnglish) {
    u8g2Fonts.setFont(enFont);
    int w = u8g2Fonts.getUTF8Width(enText);
    u8g2Fonts.setCursor(rx - w, y); u8g2Fonts.print(enText);
  } else {
    u8g2Fonts.setFont(heFont);
    printHebrewRight(heText, rx, y);
  }
}
// ---- NEW: bilingual left-anchored helper ----
// Mirrors printBilingualRight, but left-anchored -- needed for several
// of this session's fixes (Weather's humidity/wind/forecast labels,
// Settings' section headers and row labels) where the existing
// left-anchored layout needs a language branch that didn't exist before.
void printBilingualLeft(const char* heText, const char* enText, int lx, int y,
                         const uint8_t* heFont, const uint8_t* enFont) {
  if (languageEnglish) {
    u8g2Fonts.setFont(enFont);
    u8g2Fonts.setCursor(lx, y); u8g2Fonts.print(enText);
  } else {
    u8g2Fonts.setFont(heFont);
    printHebrewLeft(heText, lx, y);
  }
}

// ---- Hebrew numeral conversion (for the halachic time display) ----
String hebrewNumeral(int n) {
  if (languageEnglish) return String(n);
  const char* units[] = {"", "א","ב","ג","ד","ה","ו","ז","ח","ט"};
  const char* tens[]  = {"", "י","כ","ל","מ","נ"};
  if (n <= 0) return "";
  if (n == 15) return "טו";
  if (n == 16) return "טז";
  if (n <= 9) return String(units[n]);
  int t = n/10, u = n%10;
  return String(tens[t]) + String(units[u]);
}

// ---- Wall-clock formatting for Screen 3 (Zmanim List) ----
void epochToLocalHM(time_t trueUtcEpoch, int *outHour, int *outMin) {
  time_t shifted = trueUtcEpoch + GMT_OFFSET_SEC + DST_OFFSET_SEC;
  struct tm tmv; gmtime_r(&shifted, &tmv);
  *outHour = tmv.tm_hour;
  *outMin  = tmv.tm_min;
}
String formatWallClockHebrew(time_t trueUtcEpoch) {
  if (trueUtcEpoch <= 0) return "—";
  int h, m; epochToLocalHM(trueUtcEpoch, &h, &m);
  return hebrewNumeral(h) + ":" + (m == 0 ? String("א") : hebrewNumeral(m));
}
// ---- NEW: plain-digit wall-clock time formatter (2026-07-13) ----
// Companion to formatWallClockHebrew() above, for English mode. Reuses
// the same epochToLocalHM() extraction; returns 12-HOUR "H:MM" notation
// (no AM/PM suffix shown on screen, matching Andrew's examples of
// "3:14" and "8:10" rather than "3:14 PM"/"8:10 PM") with plain Arabic
// digits and a zero-padded minute field. Added because Andrew asked for
// English mode across several screens (Screen 1 hero time, Screen 2
// wall-clock dial, Zmanim List, candle lighting) to show ordinary
// wall-clock time instead of halachic-hour notation, while Hebrew mode
// keeps the halachic convention on those same screens.
//
// BUG FIX 2026-07-13: this originally returned 24-HOUR (military) time
// (e.g. "20:10" for 8:10 PM) -- confirmed wrong per Andrew's explicit
// feedback ("it should say 8:10 instead of 20:10"). Fixed with a real,
// verified 24->12 hour conversion: hour 0 (midnight) displays as 12;
// hours 1-12 display unchanged; hours 13-23 subtract 12.
String epochToWallClockTime(time_t trueUtcEpoch) {
  if (trueUtcEpoch <= 0) return "—";
  int h24, m; epochToLocalHM(trueUtcEpoch, &h24, &m);
  int h12 = h24;
  if (h12 == 0) h12 = 12;
  else if (h12 > 12) h12 -= 12;
  char buf[8];
  snprintf(buf, sizeof(buf), "%d:%02d", h12, m);
  return String(buf);
}

// ---- WiFi ----
bool wifiConnect() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(300);
  WiFi.setSleep(false);
  WiFi.begin(wifiSsid, wifiPassword);
  int a=0; while (WiFi.status()!=WL_CONNECTED && a<60){delay(250);a++;}
  return WiFi.status()==WL_CONNECTED;
}
void wifiOff(){ WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }

// ---- WiFi / Location Setup (captive portal) ----
WebServer provisionServer(80);
DNSServer provisionDns;
bool provisionSubmitted = false;

const char* PROVISION_FORM_HTML =
  "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Shaon Setup</title>"
  "<style>body{font-family:sans-serif;max-width:400px;margin:24px auto;padding:0 16px;}"
  "h2{margin-bottom:4px;}label{display:block;margin-top:16px;font-weight:bold;}"
  "input{width:100%%;padding:8px;font-size:16px;box-sizing:border-box;margin-top:4px;}"
  "button{margin-top:24px;width:100%%;padding:12px;font-size:16px;background:#222;color:#fff;border:none;border-radius:4px;}"
  "</style></head><body>"
  "<h2>&#1513;&#1506;&#1493;&#1503; Setup</h2>"
  "<p>Enter your WiFi network and zip code below.</p>"
  "<form action='/save' method='POST'>"
  "<label>WiFi Network Name</label><input name='ssid' value='%s' maxlength='32' required>"
  "<label>WiFi Password</label><input name='pass' type='password' value='%s' maxlength='63'>"
  "<label>Zip Code</label><input name='zip' value='%s' maxlength='5' pattern='[0-9]{5}' inputmode='numeric'>"
  "<button type='submit'>Save &amp; Connect</button>"
  "</form></body></html>";

void handleProvisionRoot() {
  char page[1400];
  snprintf(page, sizeof(page), PROVISION_FORM_HTML, wifiSsid, wifiPassword, zipCode);
  provisionServer.send(200, "text/html", page);
}

void handleProvisionSave() {
  String newSsid = provisionServer.arg("ssid");
  String newPass = provisionServer.arg("pass");
  String newZip  = provisionServer.arg("zip");

  bool zipOk = newZip.length() == 0 || newZip.length() == 5;
  if (newSsid.length() == 0 || !zipOk) {
    provisionServer.send(400, "text/html",
      "<html><body><h3>Please go back and check your entries.</h3>"
      "<p>WiFi name is required. Zip code must be 5 digits or left blank.</p>"
      "<a href='/'>Back</a></body></html>");
    return;
  }

  newSsid.toCharArray(wifiSsid, sizeof(wifiSsid));
  newPass.toCharArray(wifiPassword, sizeof(wifiPassword));
  newZip.toCharArray(zipCode, sizeof(zipCode));

  prefs.putString("wifiSsid", wifiSsid);
  prefs.putString("wifiPass", wifiPassword);
  prefs.putString("zipCode", zipCode);

  provisionServer.send(200, "text/html",
    "<html><body><h3>Saved! Shaon will now reconnect.</h3>"
    "<p>You can close this page.</p></body></html>");
  provisionSubmitted = true;
}

void drawProvisioningScreen(bool waitingForSubmit);

void startProvisioning() {
  Serial.println("Entering WiFi/Location setup mode");
  provisionSubmitted = false;

  WiFi.mode(WIFI_AP);
  bool apCfgOk = WiFi.softAPConfig(SETUP_AP_IP, SETUP_AP_IP, IPAddress(255,255,255,0));
  bool apOk = WiFi.softAP(SETUP_AP_SSID);
  Serial.printf("softAPConfig()=%s  softAP()=%s  AP IP=%s\n",
    apCfgOk ? "OK" : "FAIL", apOk ? "OK" : "FAIL",
    WiFi.softAPIP().toString().c_str());

  provisionDns.start(53, "*", SETUP_AP_IP);
  provisionServer.on("/", handleProvisionRoot);
  provisionServer.on("/save", HTTP_POST, handleProvisionSave);
  provisionServer.onNotFound(handleProvisionRoot);
  provisionServer.begin();
  Serial.println("provisionServer.begin() called -- HTTP server should be listening on port 80");

  display.setFullWindow();
  display.firstPage();
  do { drawProvisioningScreen(true); } while (display.nextPage());

  unsigned long startMs = millis();
  const unsigned long PROVISION_TIMEOUT_MS = 5UL * 60UL * 1000UL;
  unsigned long lastStatusPrint = 0;
  while (!provisionSubmitted && (millis() - startMs) < PROVISION_TIMEOUT_MS) {
    provisionDns.processNextRequest();
    provisionServer.handleClient();
    if (millis() - lastStatusPrint > 5000) {
      Serial.printf("Provisioning: waiting... stations connected=%d\n", WiFi.softAPgetStationNum());
      lastStatusPrint = millis();
    }
    delay(2);
  }

  provisionServer.stop();
  provisionDns.stop();
  WiFi.softAPdisconnect(true);

  if (!provisionSubmitted) {
    Serial.println("WiFi/Location setup timed out with no submission");
  } else {
    Serial.println("WiFi/Location setup: new credentials saved");
  }
}

WebServer logServer(80);

void handleLogRoot() {
  ensureLittleFsMounted();
  if (!littleFsReady) {
    logServer.send(200, "text/plain", "LittleFS not mounted -- no log available this session.\n");
    return;
  }
  File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
  if (!f) {
    logServer.send(200, "text/plain", "No log file yet (no wake has completed since last flash/format).\n");
    return;
  }
  logServer.setContentLength(f.size());
  logServer.send(200, "text/plain", "");
  WiFiClient client = logServer.client();
  uint8_t buf[512];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    client.write(buf, n);
  }
  f.close();
}

void handleLogNotFound() {
  logServer.sendHeader("Location", "/log", true);
  logServer.send(302, "text/plain", "");
}

void startLogServer() {
  Serial.println("Entering debug-log server mode");

  WiFi.mode(WIFI_AP);
  bool apCfgOk = WiFi.softAPConfig(SETUP_AP_IP, SETUP_AP_IP, IPAddress(255,255,255,0));
  bool apOk = WiFi.softAP(SETUP_AP_SSID);
  Serial.printf("Log server: softAPConfig()=%s  softAP()=%s  AP IP=%s\n",
    apCfgOk ? "OK" : "FAIL", apOk ? "OK" : "FAIL",
    WiFi.softAPIP().toString().c_str());

  DNSServer logDns;
  logDns.start(53, "*", SETUP_AP_IP);
  int logFetchCount = 0;
  logServer.on("/log", HTTP_GET, [&logFetchCount](){ handleLogRoot(); logFetchCount++; });
  logServer.onNotFound(handleLogNotFound);
  logServer.begin();

  unsigned long startMs = millis();
  const unsigned long LOG_SERVER_TIMEOUT_MS = 3UL * 60UL * 1000UL;
  unsigned long lastScreenUpdate = 0;
  while ((millis() - startMs) < LOG_SERVER_TIMEOUT_MS) {
    logDns.processNextRequest();
    logServer.handleClient();
    if (millis() - lastScreenUpdate > 5000) {
      unsigned long remainingSec = (LOG_SERVER_TIMEOUT_MS - (millis() - startMs)) / 1000;
      display.setFullWindow();
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        u8g2Fonts.setFont(u8g2_font_helvB12_tf);
        printPlainCentered("Debug Log Server", 100, 30);
        display.drawLine(8, 40, 192, 40, GxEPD_BLACK);
        u8g2Fonts.setFont(u8g2_font_5x7_tf);
        u8g2Fonts.setCursor(12, 60);  u8g2Fonts.print("1. Connect to WiFi:");
        u8g2Fonts.setFont(u8g2_font_helvB12_tf);
        printPlainCentered(SETUP_AP_SSID, 100, 78);
        u8g2Fonts.setFont(u8g2_font_5x7_tf);
        u8g2Fonts.setCursor(12, 100); u8g2Fonts.print("2. Visit this address:");
        u8g2Fonts.setFont(u8g2_font_helvB12_tf);
        printPlainCentered("192.168.4.1/log", 100, 120);
        u8g2Fonts.setFont(u8g2_font_5x7_tf);
        char statusLine[48];
        snprintf(statusLine, sizeof(statusLine), "Served %d time(s). %lus left.", logFetchCount, remainingSec);
        u8g2Fonts.setCursor(12, 190); u8g2Fonts.print(statusLine);
      } while (display.nextPage());
      lastScreenUpdate = millis();
    }
    delay(2);
  }

  logServer.stop();
  logDns.stop();
  WiFi.softAPdisconnect(true);
  Serial.printf("Debug log server: window elapsed, served %d time(s) total\n", logFetchCount);
}

// ---- NEW: fetch the Torah Quotes CSV from GitHub (2026-07-13) ----
// Called once per successful sunset rollover (see setup()), reusing the
// WiFi connection already open at that point -- no new trigger
// mechanism needed, since the rollover already fires once per Jewish
// calendar day. Overwrites the LittleFS cache ONLY on a successful
// fetch; a failed fetch leaves the previous day's cached copy in place
// (fetch-then-replace, never delete-then-fetch), so a network blip
// costs at most one stale day of quotes rather than none at all.
bool fetchTorahQuotesCSV() {
  ensureLittleFsMounted();
  if (!littleFsReady) {
    Serial.println("fetchTorahQuotesCSV: LittleFS not mounted, skipping");
    return false;
  }

  HTTPClient http;
  http.begin(TORAH_CSV_URL);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("Torah quotes CSV HTTP %d -- keeping previous cached copy\n", code);
    http.end();
    return false;
  }

  // Stream directly to a temp file, then rename over the real one only
  // once the full download has succeeded -- avoids leaving a
  // half-written file in place if the connection drops mid-transfer.
  const char* tmpPath = "/torah_quotes.csv.tmp";
  File f = LittleFS.open(tmpPath, FILE_WRITE);
  if (!f) {
    Serial.println("fetchTorahQuotesCSV: failed to open temp file for write");
    http.end();
    return false;
  }
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[512];
  size_t total = 0;
  while (http.connected() && (http.getSize() > 0 || http.getSize() == -1)) {
    size_t avail = stream->available();
    if (avail == 0) { delay(2); continue; }
    size_t n = stream->readBytes(buf, min(avail, sizeof(buf)));
    if (n == 0) break;
    f.write(buf, n);
    total += n;
    if (http.getSize() > 0 && total >= (size_t)http.getSize()) break;
  }
  f.close();
  http.end();

  if (total == 0) {
    Serial.println("fetchTorahQuotesCSV: downloaded 0 bytes -- keeping previous cached copy");
    LittleFS.remove(tmpPath);
    return false;
  }

  // Swap the temp file into place, replacing any previous cached copy.
  LittleFS.remove(TORAH_CSV_PATH);
  LittleFS.rename(tmpPath, TORAH_CSV_PATH);
  Serial.printf("fetchTorahQuotesCSV: downloaded %d bytes, cached successfully\n", (int)total);
  return true;
}

// ---- NEW: look up today's quote from the cached CSV (2026-07-13) ----
// Streams the file line-by-line from LittleFS rather than buffering the
// whole file into a String -- keeps peak RAM usage to roughly one
// line's width regardless of total file size (hundreds of rows once
// fully populated). Schema: semicolon-delimited (not comma), chosen
// specifically so quote text can contain literal commas without needing
// any quote-escaping logic -- see field order below.
//   ParshaName;Day;ParshaHebrew;AliyahRange;QuoteHebrew;QuoteEnglish
// Only ParshaName, Day, QuoteHebrew, QuoteEnglish are read at runtime;
// ParshaHebrew/AliyahRange exist purely as human-editing aids in the
// source file and are skipped here.
// ---- NEW: Hebrew parasha name lookup table (2026-07-13) ----
// torahParashaName (fetched in fetchNextHoliday()) is deliberately the
// BARE ENGLISH name Hebcal itself returns (e.g. "Devarim"), since that
// string is also the CSV lookup key -- it was never meant to double as
// a Hebrew display string. This table provides the actual Hebrew name
// for display purposes on the Torah Quotes header and the Calendar
// Info candle-lighting occasion line, both of which previously showed
// the English name even in Hebrew mode (a real gap, not a stylistic
// choice -- flagged explicitly in this file's own prior comments).
// All 61 entries (47 single parshas + one row-set for each of the 7
// variable-combination pairs, both combined AND split forms) are
// copied directly from the already-validated data/torah_quotes.csv
// template used for the Torah Quotes feature, so the Hebrew spellings
// here are guaranteed consistent with that file rather than retyped
// independently.
struct ParshaNamePair { const char* en; const char* he; };
static const ParshaNamePair PARSHA_NAME_TABLE[] = {
  {"Bereshit", "בְּרֵאשִׁית"},
  {"Noach", "נֹחַ"},
  {"Lech-Lecha", "לֶךְ־לְךָ"},
  {"Vayera", "וַיֵּרָא"},
  {"Chayei Sara", "חַיֵּי שָׂרָה"},
  {"Toldot", "תּוֹלְדֹת"},
  {"Vayetzei", "וַיֵּצֵא"},
  {"Vayishlach", "וַיִּשְׁלַח"},
  {"Vayeshev", "וַיֵּשֶׁב"},
  {"Miketz", "מִקֵּץ"},
  {"Vayigash", "וַיִּגַּשׁ"},
  {"Vayechi", "וַיְחִי"},
  {"Shemot", "שְׁמוֹת"},
  {"Vaera", "וָאֵרָא"},
  {"Bo", "בֹּא"},
  {"Beshalach", "בְּשַׁלַּח"},
  {"Yitro", "יִתְרוֹ"},
  {"Mishpatim", "מִּשְׁפָּטִים"},
  {"Terumah", "תְּרוּמָה"},
  {"Tetzaveh", "תְּצַוֶּה"},
  {"Ki Tisa", "כִּי תִשָּׂא"},
  {"Vayakhel-Pekudei", "וַיַּקְהֵל־פְקוּדֵי"},
  {"Vayakhel", "וַיַּקְהֵל"},
  {"Pekudei", "פְקוּדֵי"},
  {"Vayikra", "וַיִּקְרָא"},
  {"Tzav", "צַו"},
  {"Shmini", "שְּׁמִינִי"},
  {"Tazria-Metzora", "תַזְרִיעַ־מְּצֹרָע"},
  {"Tazria", "תַזְרִיעַ"},
  {"Metzora", "מְּצֹרָע"},
  {"Achrei Mot-Kedoshim", "אַחֲרֵי מוֹת־קְדֹשִׁים"},
  {"Achrei Mot", "אַחֲרֵי מוֹת"},
  {"Kedoshim", "קְדֹשִׁים"},
  {"Emor", "אֱמֹר"},
  {"Behar-Bechukotai", "בְּהַר־בְּחֻקֹּתַי"},
  {"Behar", "בְּהַר"},
  {"Bechukotai", "בְּחֻקֹּתַי"},
  {"Bamidbar", "בְּמִדְבַּר"},
  {"Nasso", "נָשֹׂא"},
  {"Beha'alotcha", "בְּהַעֲלֹתְךָ"},
  {"Sh'lach", "שְׁלַח"},
  {"Korach", "קֹרַח"},
  {"Chukat-Balak", "חֻקַּת־בָּלָק"},
  {"Chukat", "חֻקַּת"},
  {"Balak", "בָּלָק"},
  {"Pinchas", "פִּינְחָס"},
  {"Matot-Masei", "מַטּוֹת־מַסְעֵי"},
  {"Matot", "מַטּוֹת"},
  {"Masei", "מַסְעֵי"},
  {"Devarim", "דְּבָרִים"},
  {"Vaetchanan", "וָאֶתְחַנַּן"},
  {"Eikev", "עֵקֶב"},
  {"Re'eh", "רְאֵה"},
  {"Shoftim", "שֹׁפְטִים"},
  {"Ki Teitzei", "כִּי־תֵצֵא"},
  {"Ki Tavo", "כִּי־תָבוֹא"},
  {"Nitzavim-Vayeilech", "נִצָּבִים־וַיֵּלֶךְ"},
  {"Nitzavim", "נִצָּבִים"},
  {"Vayeilech", "וַיֵּלֶךְ"},
  {"Ha'azinu", "הַאֲזִינוּ"},
  {"Vezot Haberakhah", "וְזֹאת הַבְּרָכָה"},
};
static const int PARSHA_NAME_TABLE_SIZE = sizeof(PARSHA_NAME_TABLE) / sizeof(PARSHA_NAME_TABLE[0]);

// Looks up the Hebrew name for a given bare English parasha name (e.g.
// "Devarim" -> "דְּבָרִים"). Returns the English name unchanged if not
// found in the table -- an honest fallback rather than showing nothing,
// in case Hebcal ever returns a name variant not yet in this table
// (e.g. an edge case not caught by the two years of live data checked
// when this table was built).
String parshaNameToHebrew(const String& enName) {
  for (int i = 0; i < PARSHA_NAME_TABLE_SIZE; i++) {
    if (enName == PARSHA_NAME_TABLE[i].en) {
      return String(PARSHA_NAME_TABLE[i].he);
    }
  }
  return enName; // fallback: unrecognized name, show as-is rather than blank
}

bool findTodayQuote(const String& parshaName, int day, String& outHe, String& outEn) {
  ensureLittleFsMounted();
  if (!littleFsReady) return false;

  File f = LittleFS.open(TORAH_CSV_PATH, FILE_READ);
  if (!f) {
    Serial.println("findTodayQuote: no cached CSV file yet");
    return false;
  }

  bool skippedHeader = false;
  String dayStr = String(day);

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (!skippedHeader) { skippedHeader = true; continue; }

    // Manual split on ';' -- fields[0]=ParshaName, [1]=Day, [4]=QuoteHebrew, [5]=QuoteEnglish
    int idx[6]; int fieldStart = 0; int fieldNum = 0;
    String fields[6];
    for (int i = 0; i <= (int)line.length() && fieldNum < 6; i++) {
      if (i == (int)line.length() || line[i] == ';') {
        fields[fieldNum++] = line.substring(fieldStart, i);
        fieldStart = i + 1;
      }
    }
    if (fieldNum < 6) continue; // malformed row, skip rather than crash

    if (fields[0] == parshaName && fields[1] == dayStr) {
      outHe = fields[4];
      outEn = fields[5];
      f.close();
      return outHe.length() > 0 || outEn.length() > 0; // honest: row exists but may still be unfilled
    }
  }
  f.close();
  return false;
}

// ---- NEW: today's day-of-week, 1(Sun)-7(Sat) (2026-07-13) ----
// Reuses the exact days-since-epoch mod-7 pattern already established
// in fetchWeather()'s forecastWeekday[] computation, just shifted by +1
// to match this screen's 1-7 convention instead of that function's 0-6.
int computeTodayDayOfWeek(time_t nowUTC) {
  long daysSinceEpoch = nowUTC / 86400L;
  int weekday0to6 = (int)(((daysSinceEpoch % 7) + 7 + 4) % 7); // 1970-01-01 was Thursday(4), Sunday=0
  return weekday0to6 + 1; // 1=Sunday .. 7=Saturday
}

int readBatteryPct() {
  pinMode(USB_DET_PIN, INPUT);
  bool usbConnected = (digitalRead(USB_DET_PIN) == 1);
  time_t now; time(&now);

  if (usbConnected) {
    lastUsbConnectedEpoch = now;
    wasUsbConnected = true;
    haveUsbBaseline = true;
    Serial.println("Battery: USB connected -> reporting 100%");
    return 100;
  }

  if (wasUsbConnected) {
    Serial.println("Battery: USB just disconnected -- starting discharge countdown from now");
    lastUsbConnectedEpoch = now;
    wasUsbConnected = false;
  }

  if (!haveUsbBaseline) {
    Serial.println("Battery: no USB baseline yet -- charge level unknown");
    return -1;
  }

  double elapsedSec = difftime(now, lastUsbConnectedEpoch);
  double frac = 1.0 - (elapsedSec / (double)ASSUMED_BATTERY_LIFE_SECONDS);
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int pct = (int)(frac * 100.0);
  Serial.printf("Battery: %.1f hours since USB disconnected -> estimated %d%%\n",
    elapsedSec / 3600.0, pct);
  return pct;
}

bool isCharging() {
  pinMode(CHRG_STATUS_PIN, INPUT_PULLUP);
  pinMode(USB_DET_PIN, INPUT);
  int chrgRaw = digitalRead(CHRG_STATUS_PIN);
  int usbRaw  = digitalRead(USB_DET_PIN);
  Serial.printf("CHRG_STATUS_PIN(10) raw=%d   USB_DET_PIN(21) raw=%d\n", chrgRaw, usbRaw);
  return usbRaw == 1;
}

static time_t utcTmToEpoch(int y, int mo, int d, int h, int mi, int s) {
  int yy = y - (mo <= 2 ? 1 : 0);
  long era = (yy >= 0 ? yy : yy - 399) / 400;
  unsigned yoe = (unsigned)(yy - era * 400);
  unsigned mp = (mo + 9) % 12;
  unsigned doy = (153 * mp + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long daysSinceEpoch = era * 146097L + (long)doe - 719468L;
  return (time_t)daysSinceEpoch * 86400L + h * 3600L + mi * 60L + s;
}

time_t parseISOToEpoch(const char* iso) {
  int y,mo,d,h,mi,s;
  char offSign = '+'; int offH = 0, offM = 0;
  int n = sscanf(iso, "%d-%d-%dT%d:%d:%d%c%d:%d",
                 &y,&mo,&d,&h,&mi,&s,&offSign,&offH,&offM);
  if (n < 6) return 0;
  time_t utcIfFieldsWereUTC = utcTmToEpoch(y,mo,d,h,mi,s);
  long offsetSec = 0;
  if (n >= 9) {
    offsetSec = offH * 3600L + offM * 60L;
    if (offSign == '-') offsetSec = -offsetSec;
  }
  return utcIfFieldsWereUTC - offsetSec;
}

float calcMoonFraction(int year, int month, int day) {
  int a = (14 - month) / 12;
  long y = year + 4800 - a;
  int m = month + 12 * a - 3;
  long jdn = day + (153L * m + 2) / 5 + 365L * y + y / 4 - y / 100 + y / 400 - 32045;
  double daysSinceNew = (double)jdn - 2451550.26;
  double newMoons = daysSinceNew / 29.530588853;
  double frac = newMoons - floor(newMoons);
  if (frac < 0) frac += 1.0;
  return (float)frac;
}

float moonIllumination(float frac) {
  return (float)((1.0 - cos(frac * 2.0 * M_PI)) / 2.0);
}

#define GEONAMEID 5380748
void appendLocationParam(char* buf, size_t bufSize) {
  if (zipCode[0] != '\0') {
    snprintf(buf, bufSize, "zip=%s", zipCode);
  } else {
    snprintf(buf, bufSize, "geonameid=%d", GEONAMEID);
  }
}
bool fetchZmanim(int gy,int gm,int gd) {
  char locParam[24];
  appendLocationParam(locParam, sizeof(locParam));
  HTTPClient http; char url[224];
  snprintf(url,sizeof(url),
    "https://www.hebcal.com/zmanim?cfg=json&%s&date=%04d-%02d-%02d",
    locParam, gy, gm, gd);
  Serial.printf("Zmanim URL: %s\n", url);
  http.begin(url);
  int code = http.GET();
  if (code != 200){ Serial.printf("Zmanim HTTP %d\n",code); http.end(); return false; }

  String payload = http.getString();
  http.end();

  JsonDocument filter;
  filter["times"]["sunrise"]      = true;
  filter["times"]["sunset"]       = true;
  filter["times"]["alotHaShachar"] = true;
  filter["times"]["sofZmanShma"]   = true;
  filter["times"]["sofZmanShmaMGA"] = true;
  filter["times"]["chatzot"]       = true;
  filter["times"]["minchaGedola"]  = true;
  filter["times"]["tzeit85deg"]    = true;
  filter["location"]["latitude"]  = true;
  filter["location"]["longitude"] = true;
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload,
                                           DeserializationOption::Filter(filter));
  if (e){ Serial.printf("Zmanim JSON err: %s\n", e.c_str()); return false; }

  const char* sr = doc["times"]["sunrise"];
  const char* ss = doc["times"]["sunset"];
  if (!sr || !ss){ Serial.println("Zmanim missing sunrise/sunset"); return false; }
  sunriseEpoch = parseISOToEpoch(sr);
  sunsetEpoch  = parseISOToEpoch(ss);

  if (!doc["location"]["latitude"].isNull() && !doc["location"]["longitude"].isNull()) {
    currentLatitude  = doc["location"]["latitude"]  | currentLatitude;
    currentLongitude = doc["location"]["longitude"] | currentLongitude;
    Serial.printf("Location updated from Hebcal: lat=%.4f lon=%.4f\n", currentLatitude, currentLongitude);
  }

  const char* alot   = doc["times"]["alotHaShachar"];
  const char* shma    = doc["times"]["sofZmanShma"];
  const char* shmaMGA  = doc["times"]["sofZmanShmaMGA"];
  const char* chatzot = doc["times"]["chatzot"];
  const char* mgedola  = doc["times"]["minchaGedola"];
  const char* tzeit    = doc["times"]["tzeit85deg"];
  alotEpoch         = alot    ? parseISOToEpoch(alot)    : 0;
  sofZmanShmaEpoch  = shma    ? parseISOToEpoch(shma)    : 0;
  sofZmanShmaMGAEpoch = shmaMGA ? parseISOToEpoch(shmaMGA) : 0;
  chatzotEpoch      = chatzot ? parseISOToEpoch(chatzot) : 0;
  minchaGedolaEpoch = mgedola ? parseISOToEpoch(mgedola) : 0;
  tzeitEpoch        = tzeit   ? parseISOToEpoch(tzeit)   : 0;

  Serial.printf("Sunrise: %s (%ld)\nSunset: %s (%ld)\n", sr, sunriseEpoch, ss, sunsetEpoch);
  return true;
}

bool fetchOmerDay(int gy,int gm,int gd) {
  HTTPClient http; char url[256]; char dateStr[16];
  snprintf(dateStr,sizeof(dateStr),"%04d-%02d-%02d",gy,gm,gd);
  snprintf(url,sizeof(url),
    "https://www.hebcal.com/hebcal?cfg=json&v=1&start=%s&end=%s&o=on&maj=off&min=off&mod=off&nx=off&ss=off&mf=off",
    dateStr,dateStr);
  http.begin(url);
  int code = http.GET();
  if (code != 200){ Serial.printf("Omer HTTP %d\n",code); http.end(); return false; }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload);
  if (e){ Serial.printf("Omer JSON err: %s\n", e.c_str()); return false; }

  omerDay = 0;
  JsonArray items = doc["items"].as<JsonArray>();
  for (JsonObject item : items) {
    const char* cat = item["category"];
    if (cat && strcmp(cat, "omer") == 0) {
      const char* countEn = item["omer"]["count"]["en"];
      int day = 0;
      if (countEn && sscanf(countEn, "Today is %d", &day) == 1) {
        omerDay = day;
      }
      break;
    }
  }
  Serial.printf("Omer day: %d\n", omerDay);
  return true;
}

// ---- Fetch the next upcoming Jewish holiday AND this week's parasha ----
// (Screen 4 holiday countdown, and Screen 4/Torah Quotes parasha name)
// MODIFIED 2026-07-13: now also captures category=="parashat" items in
// the same response, reusing this existing 120-day-window fetch rather
// than a separate network round-trip. Also now captures "title" on
// holiday items (English name, e.g. "Shavuot I") alongside "hebrew" --
// previously only "hebrew" was fetched, leaving the holiday name
// Hebrew-only even in English display mode.
bool fetchNextHoliday(int gy,int gm,int gd, time_t nowUTC) {
  time_t startEpoch = utcTmToEpoch(gy,gm,gd,0,0,0);
  time_t endEpochT   = startEpoch + 120L*86400L;
  struct tm endTm; time_t endEpochCopy = endEpochT; gmtime_r(&endEpochCopy, &endTm);
  char startStr[16], endStr[16];
  snprintf(startStr,sizeof(startStr),"%04d-%02d-%02d",gy,gm,gd);
  snprintf(endStr,sizeof(endStr),"%04d-%02d-%02d",endTm.tm_year+1900,endTm.tm_mon+1,endTm.tm_mday);

  HTTPClient http; char url[360];
  char locParam[24];
  appendLocationParam(locParam, sizeof(locParam));
  // s=on already enables parashat items in the response (confirmed via
  // real Hebcal API docs/examples: category=="parashat" items appear
  // whenever s=on is set, alongside the candle-lighting memo behavior
  // this flag was originally added for).
  snprintf(url,sizeof(url),
    "https://www.hebcal.com/hebcal?cfg=json&v=1&start=%s&end=%s&maj=on&min=on&mod=off&nx=off&ss=off&mf=off&o=off&s=on&leyning=off"
    "&%s&c=on",
    startStr, endStr, locParam);
  http.begin(url);
  int code = http.GET();
  if (code != 200){ Serial.printf("Holiday HTTP %d\n",code); http.end(); return false; }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload);
  if (e){ Serial.printf("Holiday JSON err: %s\n", e.c_str()); return false; }

  haveNextHoliday = false;
  haveNextCandle = false;
  haveTorahParasha = false;
  // Strict 6-day (not 7-day) forward bound for the parasha window --
  // if today IS the Shabbat this parasha is read on, a 7-day window
  // would ALSO include next Saturday's parasha item, relying on
  // response ordering + the haveTorahParasha first-match guard to pick
  // the right one. A 6-day bound removes that ambiguity outright: it
  // can only ever contain the current week's single Shabbat.
  time_t parashaWindowEnd = startEpoch + 6L*86400L;

  JsonArray items = doc["items"].as<JsonArray>();
  for (JsonObject item : items) {
    const char* cat = item["category"];
    if (!cat) continue;

    if (!haveNextHoliday && strcmp(cat, "holiday") == 0) {
      const char* dateStr = item["date"];
      int hy,hm,hd;
      if (dateStr && sscanf(dateStr, "%d-%d-%d", &hy,&hm,&hd) == 3) {
        time_t itemEpoch = utcTmToEpoch(hy,hm,hd,0,0,0);
        if (itemEpoch > startEpoch) {
          const char* heb = item["hebrew"];
          const char* titleEn = item["title"];
          nextHolidayY = hy; nextHolidayM = hm; nextHolidayD = hd;
          strncpy(nextHolidayHebrew, heb ? heb : "", sizeof(nextHolidayHebrew)-1);
          strncpy(nextHolidayEn, titleEn ? titleEn : "", sizeof(nextHolidayEn)-1);
          haveNextHoliday = true;
        }
      }
    }

    if (!haveNextCandle && strcmp(cat, "candles") == 0) {
      const char* dateStr = item["date"];
      if (dateStr) {
        time_t itemEpoch = parseISOToEpoch(dateStr);
        if (itemEpoch > nowUTC) {
          nextCandleEpoch = itemEpoch;
          const char* memo = item["memo"];
          strncpy(nextCandleMemo, memo ? memo : "", sizeof(nextCandleMemo)-1);
          haveNextCandle = true;
        }
      }
    }

    if (!haveTorahParasha && strcmp(cat, "parashat") == 0) {
      const char* dateStr = item["date"];
      int py, pm, pd;
      if (dateStr && sscanf(dateStr, "%d-%d-%d", &py,&pm,&pd) == 3) {
        time_t itemEpoch = utcTmToEpoch(py,pm,pd,0,0,0);
        if (itemEpoch >= startEpoch && itemEpoch <= parashaWindowEnd) {
          const char* titleFull = item["title"]; // e.g. "Parashat Bereshit"
          const char* bare = titleFull;
          if (titleFull && strncmp(titleFull, "Parashat ", 9) == 0) {
            bare = titleFull + 9;
          }
          strncpy(torahParashaName, bare ? bare : "", sizeof(torahParashaName)-1);
          haveTorahParasha = true;
        }
      }
    }

    if (haveNextHoliday && haveNextCandle && haveTorahParasha) break; // items are date-ordered
  }
  Serial.printf("Next holiday: %s / %s (%d-%02d-%02d), found=%s\n",
    nextHolidayHebrew, nextHolidayEn, nextHolidayY, nextHolidayM, nextHolidayD,
    haveNextHoliday ? "yes" : "no");
  Serial.printf("Next candle lighting: epoch=%ld, memo='%s', found=%s\n",
    (long)nextCandleEpoch, nextCandleMemo, haveNextCandle ? "yes" : "no");
  Serial.printf("This week's parasha: '%s', found=%s\n",
    torahParashaName, haveTorahParasha ? "yes" : "no");
  return haveNextHoliday;
}

int daysUntilNextHoliday(time_t nowUTC) {
  if (!haveNextHoliday) return -1;
  time_t holidayEpoch = utcTmToEpoch(nextHolidayY, nextHolidayM, nextHolidayD, 0, 0, 0);
  double diffDays = (double)(holidayEpoch - nowUTC) / 86400.0;
  return (int)ceil(diffDays);
}

bool fetchWeather() {
  HTTPClient http; char url[320];
  snprintf(url,sizeof(url),
    "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
    "&current=temperature_2m,relative_humidity_2m,wind_speed_10m,wind_direction_10m,weather_code"
    "&daily=weather_code,temperature_2m_max&temperature_unit=%s&wind_speed_unit=mph"
    "&forecast_days=%d&timezone=auto",
    currentLatitude, currentLongitude, tempFahrenheit ? "fahrenheit" : "celsius", FORECAST_DAYS + 1);
  http.begin(url);
  int code = http.GET();
  if (code != 200){ Serial.printf("Weather HTTP %d\n",code); http.end(); return false; }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload);
  if (e){ Serial.printf("Weather JSON err: %s\n", e.c_str()); return false; }

  weatherTemp     = doc["current"]["temperature_2m"]      | 0.0;
  weatherHumidity = doc["current"]["relative_humidity_2m"] | 0;
  weatherWindSpeed = doc["current"]["wind_speed_10m"]      | 0.0;
  weatherWindDir  = doc["current"]["wind_direction_10m"]   | 0;
  weatherCode     = doc["current"]["weather_code"]         | -1;

  JsonArray dailyCode = doc["daily"]["weather_code"].as<JsonArray>();
  JsonArray dailyMax  = doc["daily"]["temperature_2m_max"].as<JsonArray>();
  JsonArray dailyTime = doc["daily"]["time"].as<JsonArray>();
  Serial.printf("Weather daily array sizes: code=%d max=%d time=%d (want %d)\n",
    (int)dailyCode.size(), (int)dailyMax.size(), (int)dailyTime.size(), FORECAST_DAYS + 1);

  for (int i = 0; i < FORECAST_DAYS; i++) {
    int srcIdx = i + 1;
    if (srcIdx < (int)dailyCode.size() && srcIdx < (int)dailyMax.size()) {
      forecastCode[i] = dailyCode[srcIdx] | -1;
      forecastTempMax[i] = (int)round((double)(dailyMax[srcIdx] | 0.0));
      const char* dateStr = dailyTime[srcIdx];
      int y,m,d;
      if (dateStr && sscanf(dateStr, "%d-%d-%d", &y,&m,&d) == 3) {
        long daysSinceEpoch = utcTmToEpoch(y,m,d,0,0,0) / 86400L;
        int weekday = (int)(((daysSinceEpoch % 7) + 7 + 4) % 7);
        forecastWeekday[i] = weekday;
      } else {
        forecastWeekday[i] = -1;
      }
      Serial.printf("  forecast[%d]: code=%d tempMax=%d weekday=%d\n",
        i, forecastCode[i], forecastTempMax[i], forecastWeekday[i]);
    } else {
      forecastCode[i] = -1;
      forecastTempMax[i] = 0;
      forecastWeekday[i] = -1;
    }
  }

  haveWeather = true;
  Serial.printf("Weather: %.1f%s code=%d humidity=%d%% wind=%.1fmph dir=%d\n",
    weatherTemp, tempFahrenheit ? "F" : "C", weatherCode, weatherHumidity, weatherWindSpeed, weatherWindDir);
  return true;
}

const char* compassDirection(int deg) {
  static const char* DIRS[8] = {"N","NE","E","SE","S","SW","W","NW"};
  int idx = (int)round(deg / 45.0) % 8;
  if (idx < 0) idx += 8;
  return DIRS[idx];
}

enum WeatherIconCat { ICON_SUN, ICON_CLOUD, ICON_RAIN };
WeatherIconCat weatherCodeToIcon(int code) {
  if (code == 0 || code == 1) return ICON_SUN;
  if (code == 2 || code == 3 || code == 45 || code == 48) return ICON_CLOUD;
  return ICON_RAIN;
}

// ---- Fetch Hebcal Hebrew date parts ----
// MODIFIED 2026-07-13: now also fetches plain numeric hy/hd fields and
// the English month name (hm) alongside heDateParts -- confirmed via
// real Hebcal API documentation that the converter endpoint returns
// these fields natively (e.g. {"hy":5771,"hm":"Iyyar","hd":29,...}).
// Needed for English-mode display of the date corner on Screen 1/Zmanim
// List, which previously always showed raw Hebrew gematria regardless
// of the language setting.
bool fetchHebrewDate(int gy,int gm,int gd) {
  HTTPClient http; char url[224];
  snprintf(url,sizeof(url),
    "https://www.hebcal.com/converter?cfg=json&gy=%d&gm=%d&gd=%d&g2h=1&strict=1",gy,gm,gd);
  http.begin(url);
  int code=http.GET();
  if (code!=200){ Serial.printf("Converter HTTP %d\n",code); http.end(); return false; }

  String payload = http.getString();
  http.end();

  JsonDocument filter;
  filter["heDateParts"]["d"] = true;
  filter["heDateParts"]["m"] = true;
  filter["heDateParts"]["y"] = true;
  filter["hy"] = true;
  filter["hm"] = true;
  filter["hd"] = true;
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload,
                                           DeserializationOption::Filter(filter));
  if (e){ Serial.printf("Converter JSON err: %s\n", e.c_str()); return false; }

  const char* d=doc["heDateParts"]["d"];
  const char* m=doc["heDateParts"]["m"];
  const char* y=doc["heDateParts"]["y"];
  if(!d||!m||!y){ Serial.println("Converter missing parts"); return false; }
  strncpy(heDay,d,sizeof(heDay)-1);
  strncpy(heMonth,m,sizeof(heMonth)-1);
  strncpy(heYear,y,sizeof(heYear)-1);

  heDayNum  = doc["hd"] | 0;
  heYearNum = doc["hy"] | 0;
  const char* hmEn = doc["hm"];
  strncpy(heMonthEn, hmEn ? hmEn : "", sizeof(heMonthEn)-1);

  Serial.printf("Hebrew date: %s %s %s (numeric: %d %s %d)\n", heDay, heMonth, heYear,
    heDayNum, heMonthEn, heYearNum);
  return true;
}

// ---- Compute current halachic time string (GRA method) ----
// ---- Halachic time, 1-indexed convention (REVERTED 2026-07-13) ----
// CONVENTION CHANGE 2026-07-13 (reverted back): Andrew tried the
// 0-indexed "sunrise = 0:00" convention earlier this session, then
// asked to switch back to the traditional 1-indexed convention
// (sunrise = hour 1, represented by aleph in Hebrew), since it makes
// displaying these times in Hebrew simpler and matches how halachic
// sources actually describe these hours ("hour 1," "hour 3," etc.).
// This function is now Hebrew-mode-focused: English mode no longer
// calls this function for display at all (Screen 1 and the Zmanim List
// now show ordinary wall-clock time in English mode -- see
// epochToWallClockTime() below), so the explicit-"0"-in-English-mode
// branch from the 0-indexed version is no longer needed here.
String computeHalachicTime(time_t now) {
  if (sunsetEpoch <= sunriseEpoch) return "—";

  if (now >= sunriseEpoch && now <= sunsetEpoch) {
    isDaytime = true;
    double dayLen = (double)(sunsetEpoch - sunriseEpoch);
    double halHour = dayLen / 12.0;
    double elapsed = (double)(now - sunriseEpoch);
    int hourNum = (int)(elapsed / halHour) + 1;
    if (hourNum > 12) hourNum = 12;
    double intoHour = fmod(elapsed, halHour);
    int minNum = (int)((intoHour / halHour) * 60.0);
    String minStr = (minNum == 0) ? (languageEnglish ? String("0") : String("א")) : hebrewNumeral(minNum);
    return hebrewNumeral(hourNum) + ":" + minStr;
  } else {
    isDaytime = false;
    double dayLen = (double)(sunsetEpoch - sunriseEpoch);
    double nightLen = 86400.0 - dayLen;
    double halHour = nightLen / 12.0;
    double elapsed;
    if (now > sunsetEpoch) {
      elapsed = (double)(now - sunsetEpoch);
    } else {
      elapsed = nightLen - (double)(sunriseEpoch - now);
      if (elapsed < 0) elapsed = 0;
    }
    int hourNum = (int)(elapsed / halHour) + 1;
    if (hourNum > 12) hourNum = 12;
    double intoHour = fmod(elapsed, halHour);
    int minNum = (int)((intoHour / halHour) * 60.0);
    String minStr = (minNum == 0) ? (languageEnglish ? String("0") : String("א")) : hebrewNumeral(minNum);
    return hebrewNumeral(hourNum) + ":" + minStr;
  }
}

// ---- Rendering ----
void drawSyncIndicator(int cx, int cy, bool fresh){
  int r = 7;
  for (int half = 0; half < 2; half++){
    float base = half * M_PI;
    float a0 = base + 0.5, a1 = base + M_PI - 0.5;
    int steps = 14;
    for (int s = 0; s < steps; s++){
      if (!fresh && (s % 2)) continue;
      float aa = a0 + (a1-a0)*s/steps;
      float ab = a0 + (a1-a0)*(s+1)/steps;
      display.drawLine(cx+cos(aa)*r, cy+sin(aa)*r, cx+cos(ab)*r, cy+sin(ab)*r, GxEPD_BLACK);
    }
    float tip = a1;
    int tx = cx+cos(tip)*r, ty = cy+sin(tip)*r;
    float perp = tip + M_PI/2;
    display.drawLine(tx, ty, tx+cos(tip-0.6)*4, ty+sin(tip-0.6)*4, GxEPD_BLACK);
    display.drawLine(tx, ty, tx+cos(perp)*3 - cos(tip)*2, ty+sin(perp)*3 - sin(tip)*2, GxEPD_BLACK);
  }
  if (!fresh){
    display.drawLine(cx-r-1, cy-r-1, cx+r+1, cy+r+1, GxEPD_BLACK);
  }
}
void drawBattery(int x,int y,int pct,bool charging){
  display.drawRect(x,y,24,11,GxEPD_BLACK);
  display.fillRect(x+24,y+3,2,5,GxEPD_BLACK);
  if (pct < 0) {
    for (int xx = x+2; xx < x+22; xx += 3) {
      display.drawLine(xx, y+2, xx+6, y+8, GxEPD_BLACK);
    }
  } else {
    int w=(int)(20*(pct/100.0)); if(w>0) display.fillRect(x+2,y+2,w,7,GxEPD_BLACK);
  }
  if (charging) {
    int cx = x + 12, cy = y + 5;
    display.fillRect(cx-4, cy-4, 9, 9, GxEPD_WHITE);
    display.fillRect(cx-1, cy-3, 3, 7, GxEPD_BLACK);
    display.fillRect(cx-3, cy-1, 7, 3, GxEPD_BLACK);
  }
}
void drawMoon(int cx,int cy,int r,float frac){
  float illum = (float)((1.0 - cos(frac * 2.0 * M_PI)) / 2.0);
  bool waxing = (frac < 0.5);
  float termFactor = (float)(2.0 * illum - 1.0);

  for(int yy=-r; yy<=r; yy++){
    int hw=(int)sqrt((float)(r*r - yy*yy));
    int termX = (int)(termFactor * hw);
    for(int xx=-hw; xx<=hw; xx++){
      bool lit;
      if (waxing) lit = (xx > termX);
      else        lit = (xx < -termX);
      if (lit){
        int px=cx+xx, py=cy+yy;
        if(((px%3)==0)&&((py%3)==0)) display.drawPixel(px,py,GxEPD_BLACK);
      }
    }
  }
  for(int s=0;s<72;s++){float a0=2*M_PI*s/72,a1=2*M_PI*(s+1)/72;
    display.drawLine(cx+cos(a0)*r,cy+sin(a0)*r,cx+cos(a1)*r,cy+sin(a1)*r,GxEPD_BLACK);}
}

void drawSun(int cx,int cy,int r){
  int core = (int)(r * 0.55);
  for(int yy=-core; yy<=core; yy++)
    for(int xx=-core; xx<=core; xx++)
      if(xx*xx+yy*yy <= core*core){
        int px=cx+xx, py=cy+yy;
        if(((px%3)==0)&&((py%3)==0)) display.drawPixel(px,py,GxEPD_BLACK);
      }
  for(int s=0;s<48;s++){float a0=2*M_PI*s/48,a1=2*M_PI*(s+1)/48;
    display.drawLine(cx+cos(a0)*core,cy+sin(a0)*core,cx+cos(a1)*core,cy+sin(a1)*core,GxEPD_BLACK);}
  for(int i=0;i<12;i++){
    float a=i*(2*M_PI/12);
    int x1=cx+(int)(cos(a)*(r*0.72)), y1=cy+(int)(sin(a)*(r*0.72));
    int x2=cx+(int)(cos(a)*r),        y2=cy+(int)(sin(a)*r);
    display.drawLine(x1,y1,x2,y2,GxEPD_BLACK);
  }
}

void drawCloudIcon(int cx, int cy, int r) {
  int r1 = (int)(r * 0.42), r2 = (int)(r * 0.52), r3 = (int)(r * 0.38);
  int x1 = cx - (int)(r*0.4), y1 = cy + (int)(r*0.1);
  int x2 = cx + (int)(r*0.05), y2 = cy - (int)(r*0.15);
  int x3 = cx + (int)(r*0.5), y3 = cy + (int)(r*0.1);
  display.drawCircle(x1, y1, r1, GxEPD_BLACK);
  display.drawCircle(x2, y2, r2, GxEPD_BLACK);
  display.drawCircle(x3, y3, r3, GxEPD_BLACK);
  display.drawLine(x1 - r1/2, y1 + r1, x3 + r3/2, y3 + r3, GxEPD_BLACK);
}

void drawRainIcon(int cx, int cy, int r) {
  drawCloudIcon(cx, cy - (int)(r*0.2), (int)(r*0.8));
  for (int i = -1; i <= 1; i++) {
    int x = cx + i * (int)(r*0.4);
    display.drawLine(x, cy + (int)(r*0.35), x - (int)(r*0.12), cy + (int)(r*0.75), GxEPD_BLACK);
  }
}

void drawWeatherIcon(WeatherIconCat cat, int cx, int cy, int r) {
  switch (cat) {
    case ICON_SUN:   drawSun(cx, cy, r); break;
    case ICON_CLOUD: drawCloudIcon(cx, cy, r); break;
    case ICON_RAIN:  drawRainIcon(cx, cy, r); break;
  }
}

// ============================================================
// Screen 1 (case 0) — Halachic Digital
// MODIFIED 2026-07-13: the date corner (day/month/year) previously
// always rendered heDay/heMonth/heYear via printHebrewRight() with NO
// language check at all -- meaning it stayed Hebrew gematria even in
// English mode, a real bug rather than a missing translation. Now
// branches on languageEnglish: Hebrew mode is unchanged; English mode
// shows the day and year as Arabic numerals and the month as an English
// text string, using the numeric heDayNum/heYearNum/heMonthEn fields
// fetched in fetchHebrewDate(). Numerals use a stock digit-capable font
// (u8g2_font_helvB12_tf) rather than the frankruhl_hebrew_* fonts, which
// were confirmed (via their own documented 31-glyph subset: Hebrew
// letters + finals + geresh/gershayim + space + colon) to contain NO
// ASCII digit glyphs at all.
// ============================================================
// ---- NEW: hero halachic time renderer, splitting hour/minute segments
// (2026-07-13) ----
// Needed because the 0-indexed hour convention means computeHalachicTime()
// can return a literal "0" (Latin digit) for either segment in EITHER
// language now (hour 0 at sunrise/sunset boundary, or exactly on a
// half-hour boundary) -- gematria has no zero symbol, so "0" is used as
// an explicit fallback regardless of language. But frankruhl_hebrew_42
// (the Hebrew hero font) has no digit glyphs at all, so a mixed string
// like "0:מו" would partially fail to render if drawn with one font for
// the whole string. This function draws the string in segments, each
// using whichever font actually has the needed glyphs for that specific
// segment's content.
//
// IMPORTANT ordering detail: in Hebrew (RTL) mode, the correct visual
// result is produced by reversing the WHOLE logical "hour:minute"
// string FIRST (matching how printHebrewCentered/reverseUTF8 already
// works everywhere else in this file), THEN splitting that already-
// reversed string at its colon position to identify which physical
// character range is which segment. Reversing each segment
// INDEPENDENTLY (which an earlier version of this function did)
// produces the wrong left-right ordering -- it puts the hour on the
// visual left and minute on the right, which is backwards for RTL
// reading order. Splitting the pre-reversed whole string instead keeps
// each segment's internal letter order correct AND preserves the
// correct visual left-right placement.
//
// Font size: REVISED 2026-07-13 based on REAL measured data (not
// guessed). Diagnostic logging added in the previous fix confirmed via
// real serial output: frankruhl_hebrew_42 measures ascent=39 descent=-12
// (height=51px); u8g2_font_fub42_tf measures ascent=42 descent=-12
// (height=54px) -- only a ~6% height difference, far too small to
// explain Andrew's report that English numerals looked NOTICEABLY
// larger. This points to font WEIGHT, not height, as the real driver:
// frankruhl_hebrew_42 is a "Medium" weight font (per its own font
// name), while u8g2_font_fub42_tf is BOLD ("fub" = FreeUniversal Bold).
// A bold font at nearly the same height reads as visually heavier/
// larger due to thicker strokes. Switched to u8g2_font_fur42_tf (the
// REGULAR, non-bold weight in the same FreeUniversal family/size,
// confirmed to exist in u8g2's font library alongside fub42 in the
// same font-group listing) as a closer match to frankruhl_hebrew_42's
// medium weight. Diagnostic logging updated to measure this new font
// choice so the next test confirms with real numbers rather than
// assuming this fully resolves it.
void drawHeroHalachicTime(const String& heroTime, int cx, int cy) {
  // COMPILE FIX 2026-07-13: this local variable was previously named
  // "display", which SHADOWS the global e-paper display object (also
  // named `display`, declared at file scope) for the entire rest of
  // this function. Every `display.fillCircle(...)` call below was
  // silently being resolved against this local String variable instead
  // of the real GxEPD2 display object -- String has no fillCircle()
  // method, so this failed to compile outright (confirmed via real
  // build output: "class String has no member named fillCircle").
  // Renamed to displayStr to eliminate the collision entirely.
  String displayStr = languageEnglish ? heroTime : reverseUTF8(heroTime.c_str());
  int colonIdx = displayStr.indexOf(':');
  String leftPart  = (colonIdx >= 0) ? displayStr.substring(0, colonIdx) : displayStr;
  String rightPart = (colonIdx >= 0) ? displayStr.substring(colonIdx + 1) : String("");

  // Each part needs a digit-capable font if it's the explicit "0"
  // fallback, the "—" no-data placeholder, or if we're in English mode
  // (where every numeral is always a plain digit, never a Hebrew letter).
  auto needsDigitFont = [&](const String& seg) -> bool {
    return languageEnglish || seg == "0" || seg == "—";
  };

  const uint8_t* leftFont  = needsDigitFont(leftPart)  ? u8g2_font_fur42_tf : frankruhl_hebrew_42;
  const uint8_t* rightFont = needsDigitFont(rightPart) ? u8g2_font_fur42_tf : frankruhl_hebrew_42;

  // Real font metrics, measured via U8G2_FOR_ADAFRUIT_GFX's own
  // getFontAscent()/getFontDescent() API. Originally added purely as
  // diagnostic logging to debug a font-size mismatch (see git history);
  // the values are now genuinely load-bearing for the colon-sizing math
  // below (realFontHeight), so the measurement calls stay -- only the
  // one-time diagnostic Serial.printf() that reported these numbers
  // during development has been removed now that it served its purpose.
  u8g2Fonts.setFont(frankruhl_hebrew_42);
  int8_t heAscent = u8g2Fonts.getFontAscent();
  int8_t heDescent = u8g2Fonts.getFontDescent();
  u8g2Fonts.setFont(u8g2_font_fur42_tf);
  int8_t enAscent = u8g2Fonts.getFontAscent();
  int8_t enDescent = u8g2Fonts.getFontDescent();

  // BUG FIX 2026-07-13: previously drew the colon as the font's own ':'
  // glyph, back-to-back against the numerals with zero explicit gap on
  // either side -- any unevenness in the visual spacing came from
  // whatever left/right side-bearing the font itself baked into that
  // glyph, which differs between frankruhl_hebrew_42 and
  // u8g2_font_fub42_tf (two entirely different font files with
  // different internal metrics). Fixed by removing the font's colon
  // glyph entirely and instead: (1) reserving an explicit, EQUAL gap on
  // both sides of the colon position, and (2) hand-drawing the colon as
  // two small filled circles at a computed vertical midpoint relative
  // to the numeral height, rather than relying on the colon glyph's own
  // baseline-relative position (which, since cy is the shared text
  // baseline, was never guaranteed to sit visually centered between the
  // TOP and BOTTOM of the flanking numerals -- it was just wherever
  // each font's own colon glyph happened to be drawn).
  // CHANGE 2026-07-13: colon dots enlarged per Andrew's feedback that
  // the previous size (radius = fontSize*0.045, ~1.9px at 42px) read as
  // too small relative to the numeral height. Increased to a
  // proportionally bigger, bolder colon that reads clearly next to
  // 42px-tall numerals, while keeping the same equal-gap-on-both-sides
  // and vertical-midpoint-centering approach from the original fix.
  // Both Hebrew and English modes call this same function with the
  // same cx/cy (100, 76 from drawScreen1) -- since the colon's position
  // is computed relative to that shared cx/cy and the measured numeral
  // widths, it already occupies the same screen position in both
  // languages by construction; the size increase applies identically to
  // both.
  // BUG FIX 2026-07-13 (v2): the colon was sized using a single
  // hardcoded fontSize=42 constant for BOTH languages, which never
  // actually matched either font's REAL measured height (Hebrew: 51px,
  // English fur42_tf: 54px, per the diagnostic log above) -- 42 was
  // just a guess baked in before real measurements existed. This meant
  // the colon was smaller than intended in BOTH languages, and
  // specifically read as too small in English per Andrew's feedback
  // (English numerals are the taller of the two real measured heights,
  // so a colon sized off the wrong, smaller constant looked especially
  // undersized next to them). Fixed by using each font's OWN real
  // measured height (ascent - descent, already computed just above via
  // getFontAscent()/getFontDescent()) to size the colon -- Hebrew mode
  // scales off heAscent-heDescent (51), English mode off
  // enAscent-enDescent (54).
  //
  // ADDITIONAL FIX: height-based scaling alone only gives English a
  // ~6% larger colon than Hebrew (54 vs 51), which likely isn't enough
  // to satisfy Andrew's direct request for a NOTICEABLY larger colon in
  // English. The likely reason a proportionally-correct colon still
  // reads as small in English: u8g2_font_fur42_tf is a REGULAR
  // (non-bold) weight -- chosen last round specifically to REDUCE
  // visual weight and better match frankruhl_hebrew_42's medium weight
  // -- so its numerals have thinner strokes than a bold font would,
  // making even a height-matched colon look proportionally slighter
  // next to them.
  //
  // BUG FIX 2026-07-13 (v3): the previous fix boosted the DOT RADIUS
  // for English mode, which Andrew reports made the colon read as too
  // THICK/bulky. Per his direct feedback ("needs to be taller and the
  // circles need to be smaller"), the radius boost is removed entirely
  // -- circles are now smaller in both languages (base proportion
  // reduced from 0.075 to 0.06 of font height) -- and the "bigger
  // colon" effect for English is now achieved by boosting the
  // SEPARATION between the two dots instead (taller overall colon),
  // not by making each dot bigger.
  int realFontHeight = languageEnglish ? (enAscent - enDescent) : (heAscent - heDescent);
  const float englishSeparationBoost = languageEnglish ? 1.5f : 1.0f;
  const int gap = (int)(realFontHeight * 0.12); // equal gap on each side of the colon
  const int colonDotR = (int)(realFontHeight * 0.06);
  const int colonDotSeparation = (int)(realFontHeight * 0.26 * englishSeparationBoost);
  const int colonMidpointAboveBaseline = (int)(realFontHeight * 0.35);
  const int colonVisualWidth = colonDotR * 2 + gap * 2; // total horizontal space this colon representation occupies

  u8g2Fonts.setFont(leftFont);
  int leftW = u8g2Fonts.getUTF8Width(leftPart.c_str());

  u8g2Fonts.setFont(rightFont);
  int rightW = (colonIdx >= 0) ? u8g2Fonts.getUTF8Width(rightPart.c_str()) : 0;

  int colonSpace = (colonIdx >= 0) ? colonVisualWidth : 0;
  int totalW = leftW + colonSpace + rightW;
  int startX = cx - totalW / 2;

  u8g2Fonts.setFont(leftFont);
  u8g2Fonts.setCursor(startX, cy);
  u8g2Fonts.print(leftPart);

  if (colonIdx >= 0) {
    int colonCenterX = startX + leftW + gap + colonDotR;
    int upperDotY = cy - colonMidpointAboveBaseline - colonDotSeparation / 2;
    int lowerDotY = cy - colonMidpointAboveBaseline + colonDotSeparation / 2;
    display.fillCircle(colonCenterX, upperDotY, colonDotR, GxEPD_BLACK);
    display.fillCircle(colonCenterX, lowerDotY, colonDotR, GxEPD_BLACK);

    u8g2Fonts.setFont(rightFont);
    u8g2Fonts.setCursor(startX + leftW + colonSpace, cy);
    u8g2Fonts.print(rightPart);
  }
}

// ============================================================
// Screen 1 (case 0) — Halachic Digital
// CHANGE 2026-07-13: English mode now shows ordinary WALL-CLOCK time
// (via epochToWallClockTime()) in the hero position, rather than
// halachic-hour notation, per Andrew's explicit request. Hebrew mode
// still shows the halachic time as before (via computeHalachicTime()).
// This screen now computes its own hero time internally from `now`
// (newly passed in as a parameter) rather than trusting the shared
// `heroTime` string computed once in setup() -- that shared string is
// always halachic notation regardless of language, which no longer
// matches what this screen needs to display in English mode. Screens
// that still want the shared halachic heroTime value (there are none
// left after this change, but the parameter is kept for now in case
// other logic depends on it) are unaffected.
void drawScreen1(time_t now, const String& heroTime, int battPct, bool syncFresh, bool charging){
  display.fillScreen(GxEPD_WHITE);
  // Sync indicator moved off this screen entirely -- replaced by a
  // "Last Updated" timestamp line on the Settings screen instead (see
  // drawSettings()). Battery moved to the upper-left corner, matching
  // the corner-bracket framing convention used elsewhere.
  drawBattery(16,12,battPct,charging);
  // BUG FIX 2026-07-13: real measured font data (from the on-device
  // diagnostic log: frankruhl_hebrew_42 ascent=39, u8g2_font_fur42_tf
  // ascent=42) shows English numerals extend 3px HIGHER above the
  // shared baseline than Hebrew numerals do, at the same cy. Both
  // fonts share text-baseline positioning (cy = baseline, glyph top =
  // cy - ascent), so at the same cy, the taller-ascent English font's
  // top sits 3px above the Hebrew font's top. Shifting English mode's
  // baseline DOWN by that exact 3px difference (76 -> 79) makes the
  // GLYPH TOPS align, per Andrew's request -- computed from real
  // measurements, not estimated.
  String displayTime = languageEnglish ? epochToWallClockTime(now) : heroTime;
  int heroY = languageEnglish ? 79 : 76;
  drawHeroHalachicTime(displayTime, 100, heroY);

  if (languageEnglish) {
    char dayBuf[4], yearBuf[6];
    snprintf(dayBuf, sizeof(dayBuf), "%d", heDayNum);
    snprintf(yearBuf, sizeof(yearBuf), "%d", heYearNum);
    u8g2Fonts.setFont(u8g2_font_helvB12_tf);
    int w1 = u8g2Fonts.getUTF8Width(dayBuf);
    u8g2Fonts.setCursor(188 - w1, 128); u8g2Fonts.print(dayBuf);
    int w2 = u8g2Fonts.getUTF8Width(heMonthEn);
    u8g2Fonts.setCursor(188 - w2, 154); u8g2Fonts.print(heMonthEn);
    int w3 = u8g2Fonts.getUTF8Width(yearBuf);
    u8g2Fonts.setCursor(188 - w3, 180); u8g2Fonts.print(yearBuf);
  } else {
    u8g2Fonts.setFont(frankruhl_hebrew_18);
    printHebrewRight(heDay,188,128);
    printHebrewRight(heMonth,188,154);
    printHebrewRight(heYear,188,180);
  }

  if (isDaytime) drawSun(52,154,38);
  else           drawMoon(52,154,38,moonFrac);
}

// ============================================================
// Screen 2 (case 1) — Halachic Analog Face (Hebrew) / Wall-Clock Face (English)
// HISTORY: this screen's numbering convention changed twice this
// session -- first from 1-indexed to 0-indexed, then back to 1-indexed
// per Andrew's final preference (aleph represents hour 1, not "0").
// The current, final state as of 2026-07-13:
//   - Hebrew mode: the original halachic asymmetric-sweep dial,
//     numerals 1-12 (aleph-yud-bet), font frankruhl_hebrew_12.
//   - English mode: a SEPARATE fixed-position 24-hour wall-clock dial
//     (noon always at top, midnight always at bottom), with sunrise/
//     sunset simply falling wherever they land on that fixed dial --
//     see the languageEnglish branch inside drawHalachicAnalog() below
//     for the actual implementation and angle-formula verification.
// Also carries an earlier real bug fix (still in effect): the tick
// ANGLES were always correct (verified against the actual
// sh2_tickAngle() trig); only the printed Hebrew-dial numeral value
// needed correcting to wrap properly at the 12-hour boundary.
static const int16_t SH2_CX = 100, SH2_CY = 100, SH2_R = 92;

static float sh2_tickAngle(float i, float sunriseA, float sunsetA,
                            float daySweep, float nightSweep) {
  if (i < 12.0f) return sunriseA + (i / 12.0f) * daySweep;
  return sunsetA + ((i - 12.0f) / 12.0f) * nightSweep;
}

static void sh2_drawSolidArc(int16_t cx, int16_t cy, int16_t r,
                              float startA, float endA, int16_t lineWidth) {
  float arcLen = fabsf(endA - startA) * r;
  int steps = max(2, (int)arcLen);
  float da = (endA - startA) / steps;
  for (int w = 0; w < lineWidth; w++) {
    int16_t rr = r - (lineWidth / 2) + w;
    int16_t px = cx + (int16_t)roundf(cosf(startA) * rr);
    int16_t py = cy + (int16_t)roundf(sinf(startA) * rr);
    for (int s = 1; s <= steps; s++) {
      float a = startA + da * s;
      int16_t x = cx + (int16_t)roundf(cosf(a) * rr);
      int16_t y = cy + (int16_t)roundf(sinf(a) * rr);
      display.drawLine(px, py, x, y, GxEPD_BLACK);
      px = x; py = y;
    }
  }
}

static void sh2_drawDottedArc(int16_t cx, int16_t cy, int16_t r,
                               float startA, float endA, int16_t lineWidth,
                               int16_t dashOn = 2, int16_t dashOff = 2) {
  float arcLen = fabsf(endA - startA) * r;
  int steps = max(2, (int)arcLen);
  float da = (endA - startA) / steps;
  int period = dashOn + dashOff;
  for (int w = 0; w < lineWidth; w++) {
    int16_t rr = r - (lineWidth / 2) + w;
    for (int s = 0; s <= steps; s++) {
      if ((s % period) >= dashOn) continue;
      float a = startA + da * s;
      int16_t x = cx + (int16_t)roundf(cosf(a) * rr);
      int16_t y = cy + (int16_t)roundf(sinf(a) * rr);
      display.drawPixel(x, y, GxEPD_BLACK);
    }
  }
}

static void sh2_drawTick(int16_t cx, int16_t cy, float a,
                          int16_t rOuter, int16_t rInner, int16_t lineWidth) {
  int16_t x1 = cx + (int16_t)roundf(cosf(a) * rOuter);
  int16_t y1 = cy + (int16_t)roundf(sinf(a) * rOuter);
  int16_t x2 = cx + (int16_t)roundf(cosf(a) * rInner);
  int16_t y2 = cy + (int16_t)roundf(sinf(a) * rInner);
  for (int w = 0; w < lineWidth; w++) {
    int16_t off = w - lineWidth / 2;
    display.drawLine(x1 + off, y1, x2 + off, y2, GxEPD_BLACK);
  }
}

static float sh2_currentHourIdx(time_t now, time_t sunrise, time_t sunset,
                                 time_t nextSunrise) {
  if (now >= sunrise && now <= sunset) {
    float frac = (float)(now - sunrise) / (float)(sunset - sunrise);
    return frac * 12.0f;
  } else {
    double nightLen = 86400.0 - (double)(sunset - sunrise);
    double elapsed;
    if (now > sunset) {
      elapsed = (double)(now - sunset);
    } else {
      elapsed = nightLen - (double)(nextSunrise - now);
      if (elapsed < 0) elapsed = 0;
    }
    float frac = (float)(elapsed / (nightLen > 0 ? nightLen : 1.0));
    return 12.0f + frac * 12.0f;
  }
}

void drawHalachicAnalog(time_t now, int battPct, bool charging) {
  display.fillScreen(GxEPD_WHITE);
  // Battery icon removed from this screen per Andrew's request.
  if (sunsetEpoch <= sunriseEpoch) return;

  const float TWO_PI_F = 2.0f * PI;
  const float topA  = -PI / 2.0f;

  if (languageEnglish) {
    // ============================================================
    // NEW 2026-07-13: English-mode WALL-CLOCK dial. Per Andrew's
    // explicit request, this is a standard 24-hour dial where NOON IS
    // ALWAYS AT THE TOP (12 o'clock position) and MIDNIGHT IS ALWAYS AT
    // THE BOTTOM (6 o'clock position) -- unlike the Hebrew halachic
    // dial below, where noon/midnight position is fixed but sunrise/
    // sunset symmetrically bracket the top based on that day's actual
    // daylight length. Here, sunrise and sunset simply fall wherever
    // they actually land on this FIXED-position 24-hour dial, and the
    // dotted (day) / solid (night) arc boundary is drawn at those real
    // positions rather than being symmetric around the top.
    //
    // Angle formula verified: for secondsSinceMidnight -> angle,
    // frac = secondsSinceMidnight/86400 (0..1 through the day), and
    // angle = topA + (frac - 0.5) * 2*PI. This correctly places noon
    // (frac=0.5) at topA (-90deg/top) and midnight (frac=0 or 1) at
    // topA +/- PI (bottom), confirmed against real sunrise/sunset test
    // values before writing this code.
    auto wallClockAngle = [&](time_t epoch) -> float {
      time_t shifted = epoch + GMT_OFFSET_SEC + DST_OFFSET_SEC;
      struct tm tmv; gmtime_r(&shifted, &tmv);
      float secondsSinceMidnight = tmv.tm_hour * 3600.0f + tmv.tm_min * 60.0f + tmv.tm_sec;
      float frac = secondsSinceMidnight / 86400.0f;
      return topA + (frac - 0.5f) * TWO_PI_F;
    };

    float sunriseA = wallClockAngle(sunriseEpoch);
    float sunsetA  = wallClockAngle(sunsetEpoch);

    // Day arc (dotted) runs from sunrise to sunset the SHORT way
    // (through noon/top); night arc (solid) runs from sunset to
    // sunrise the short way (through midnight/bottom) -- same dotted/
    // solid convention as the Hebrew dial, just at fixed clock
    // positions instead of a symmetric sunrise/sunset sweep.
    sh2_drawDottedArc(SH2_CX, SH2_CY, SH2_R, sunriseA, sunsetA, 3, 2, 2);
    sh2_drawSolidArc(SH2_CX, SH2_CY, SH2_R, sunsetA, sunriseA + TWO_PI_F, 3);

    sh2_drawTick(SH2_CX, SH2_CY, sunriseA, SH2_R + 5, SH2_R - 9, 3);
    sh2_drawTick(SH2_CX, SH2_CY, sunsetA,  SH2_R + 5, SH2_R - 9, 3);

    // 24 evenly-spaced ticks around the FIXED 24-hour dial (one tick
    // per hour, hour 0 at bottom/midnight, hour 12 at top/noon).
    for (int h = 0; h < 24; h++) {
      float a = topA + ((h / 24.0f) - 0.5f) * TWO_PI_F;
      sh2_drawTick(SH2_CX, SH2_CY, a, SH2_R - 2, SH2_R - 9, 1);
    }

    // Label every 3rd hour, matching the same "every 3rd tick" labeling
    // density as the Hebrew dial. ANGLE is still computed from the raw
    // 24-hour position h (0-21), since that's what correctly places
    // each tick around the fixed dial -- only the DISPLAYED NUMBER is
    // converted to its 12-hour equivalent.
    //
    // BUG FIX 2026-07-13: previously printed the raw 24-hour value h
    // directly (0, 3, 6, 9, 12, 15, 18, 21) -- so midnight showed "0"
    // instead of "12", and afternoon/evening hours showed their 24-hour
    // form (15, 18, 21) instead of the expected 12-hour form (3, 6, 9).
    // Confirmed wrong per Andrew's explicit feedback. Fixed with the
    // same verified 24->12 conversion used in epochToWallClockTime():
    // hour 0 displays as 12; hours 1-12 unchanged; hours 13-23 subtract
    // 12.
    for (int h = 0; h < 24; h += 3) {
      float a = topA + ((h / 24.0f) - 0.5f) * TWO_PI_F;
      float lr = SH2_R - 20;
      int16_t x = SH2_CX + (int16_t)roundf(cosf(a) * lr);
      int16_t y = SH2_CY + (int16_t)roundf(sinf(a) * lr);
      int h12 = h;
      if (h12 == 0) h12 = 12;
      else if (h12 > 12) h12 -= 12;
      char buf[3];
      snprintf(buf, sizeof(buf), "%d", h12);
      u8g2Fonts.setFont(u8g2_font_6x10_tf);
      printPlainCentered(buf, x, y + 4);
    }

    // Hour hand: angle is a direct function of the current wall-clock
    // time (not day/night-relative like the Hebrew dial's hand).
    float handA = wallClockAngle(now);
    int16_t hx = SH2_CX + (int16_t)roundf(cosf(handA) * (SH2_R - 16));
    int16_t hy = SH2_CY + (int16_t)roundf(sinf(handA) * (SH2_R - 16));
    display.drawLine(SH2_CX, SH2_CY, hx, hy, GxEPD_BLACK);
    display.drawLine(SH2_CX + 1, SH2_CY, hx + 1, hy, GxEPD_BLACK);
    display.fillCircle(SH2_CX, SH2_CY, 3, GxEPD_BLACK);

    // Digital readout: ordinary wall-clock time, same position/sizing
    // convention as the Hebrew dial's digital readout (between center
    // and the numeral ring, on whichever half of the circle matches
    // day/night -- isDaytime is set as a side effect of
    // computeHalachicTime(), called via the shared heroTime computation
    // in setup(), so it's already correct by the time this runs).
    String digitalTime = epochToWallClockTime(now);
    int digitalRadius = (SH2_R - 20) / 2;
    int baseY = isDaytime ? (SH2_CY + digitalRadius) : (SH2_CY - digitalRadius);
    u8g2Fonts.setFont(u8g2_font_helvB12_tf);
    printPlainCentered(digitalTime.c_str(), SH2_CX, baseY);
    return;
  }

  // ============================================================
  // Hebrew mode: original halachic asymmetric-sweep dial (unchanged
  // geometry from earlier this session), with two reversions per
  // Andrew's latest feedback:
  //   1. Numerals back to 1-12 (aleph represents hour 1 again),
  //      matching the reverted 1-indexed computeHalachicTime()/
  //      epochToHalachicTime() convention.
  //   2. Numeral font back to frankruhl_hebrew_12 (was briefly 18px,
  //      too large per Andrew's feedback).
  // ============================================================
  float dayFraction = (float)(sunsetEpoch - sunriseEpoch) / 86400.0f;
  float daySweep    = dayFraction * TWO_PI_F;
  float nightSweep  = TWO_PI_F - daySweep;
  float sunriseA = topA - daySweep / 2.0f;
  float sunsetA  = topA + daySweep / 2.0f;

  sh2_drawDottedArc(SH2_CX, SH2_CY, SH2_R, sunriseA, sunsetA, 3, 2, 2);
  sh2_drawSolidArc(SH2_CX, SH2_CY, SH2_R, sunsetA, sunriseA + TWO_PI_F, 3);

  sh2_drawTick(SH2_CX, SH2_CY, sunriseA, SH2_R + 5, SH2_R - 9, 3);
  sh2_drawTick(SH2_CX, SH2_CY, sunsetA,  SH2_R + 5, SH2_R - 9, 3);

  for (int i = 0; i < 24; i++) {
    float a = sh2_tickAngle((float)i, sunriseA, sunsetA, daySweep, nightSweep);
    sh2_drawTick(SH2_CX, SH2_CY, a, SH2_R - 2, SH2_R - 9, 1);
  }

  for (int k = 0; k < 8; k++) {
    int i = k * 3;
    float a = sh2_tickAngle((float)i, sunriseA, sunsetA, daySweep, nightSweep);
    float lr = SH2_R - 20;
    int16_t x = SH2_CX + (int16_t)roundf(cosf(a) * lr);
    int16_t y = SH2_CY + (int16_t)roundf(sinf(a) * lr);
    // REVERTED 2026-07-13: back to 1-indexed display (tick index i=0
    // now displays as hour 1/aleph, not "0"/omitted) -- matches the
    // reverted computeHalachicTime()/epochToHalachicTime() convention.
    // Day ticks (i<12) display i+1; night ticks (i>=12) display
    // (i-12)+1.
    int displayHour = (i < 12) ? (i + 1) : (i - 12 + 1);
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    String numeral = hebrewNumeral(displayHour);
    printHebrewCentered(numeral.c_str(), x, y + 4);
  }

  float hourIdx = sh2_currentHourIdx(now, sunriseEpoch, sunsetEpoch, sunriseEpoch);
  float handA = sh2_tickAngle(hourIdx, sunriseA, sunsetA, daySweep, nightSweep);
  int16_t hx = SH2_CX + (int16_t)roundf(cosf(handA) * (SH2_R - 16));
  int16_t hy = SH2_CY + (int16_t)roundf(sinf(handA) * (SH2_R - 16));
  display.drawLine(SH2_CX, SH2_CY, hx, hy, GxEPD_BLACK);
  display.drawLine(SH2_CX + 1, SH2_CY, hx + 1, hy, GxEPD_BLACK);
  display.fillCircle(SH2_CX, SH2_CY, 3, GxEPD_BLACK);

  // Digital time readout: halachic notation (1-indexed, aleph for hour
  // 1), positioned between the dial center and the numeral ring.
  String digitalTime = haveData ? computeHalachicTime(now) : "—";
  int digitalRadius = (SH2_R - 20) / 2;
  int baseY = isDaytime ? (SH2_CY + digitalRadius) : (SH2_CY - digitalRadius);
  u8g2Fonts.setFont(frankruhl_hebrew_18);
  printHebrewCentered(digitalTime.c_str(), SH2_CX, baseY);
}

// ============================================================
// Screen 3 (case 2) — Zmanim List
// MODIFIED 2026-07-13: the short-date corner (top-left) was previously
// only drawn in Hebrew mode with no English equivalent -- a blank
// corner in English mode, per the code's own TODO comment. Now uses
// heDayNum (fetched in fetchHebrewDate()) to show the day as an Arabic
// numeral in English mode, using a digit-capable stock font.
// ============================================================
// ---- General halachic-hour conversion for an arbitrary epoch (2026-07-13) ----
// Extracted from computeHalachicTime()'s day/night branching logic so
// any zmanim epoch (not just "now") can be converted to halachic
// hour:minute notation. Needed for the Zmanim List screen: Andrew
// clarified that the times shown there ARE genuinely halachic (real
// Hebcal-computed zmanim), but were being displayed in ordinary
// wall-clock hour:minute rather than halachic-hour notation (e.g.
// Sunrise should read as the start of hour 1, not "5:58"). Dawn (Alot
// Hashachar) specifically falls BEFORE sunrise, so it needs the same
// night-side math already used for pre-sunrise/post-sunset times in
// computeHalachicTime() -- this function reuses that exact logic
// rather than assuming a single daytime-only formula would work for
// every row.
String epochToHalachicTime(time_t epoch) {
  if (epoch <= 0 || sunsetEpoch <= sunriseEpoch) return "—";

  if (epoch >= sunriseEpoch && epoch <= sunsetEpoch) {
    double dayLen = (double)(sunsetEpoch - sunriseEpoch);
    double halHour = dayLen / 12.0;
    double elapsed = (double)(epoch - sunriseEpoch);
    // REVERTED 2026-07-13: back to the 1-indexed convention (sunrise =
    // hour 1, represented by aleph), matching computeHalachicTime()'s
    // reversion. See that function's comments for the full history.
    int hourNum = (int)(elapsed / halHour) + 1;
    if (hourNum > 12) hourNum = 12;
    double intoHour = fmod(elapsed, halHour);
    int minNum = (int)((intoHour / halHour) * 60.0);
    String minStr = (minNum == 0) ? (languageEnglish ? String("0") : String("א")) : hebrewNumeral(minNum);
    return hebrewNumeral(hourNum) + ":" + minStr;
  } else {
    double dayLen = (double)(sunsetEpoch - sunriseEpoch);
    double nightLen = 86400.0 - dayLen;
    double halHour = nightLen / 12.0;
    double elapsed;
    if (epoch > sunsetEpoch) {
      elapsed = (double)(epoch - sunsetEpoch);
    } else {
      elapsed = nightLen - (double)(sunriseEpoch - epoch);
      if (elapsed < 0) elapsed = 0;
    }
    int hourNum = (int)(elapsed / halHour) + 1;
    if (hourNum > 12) hourNum = 12;
    double intoHour = fmod(elapsed, halHour);
    int minNum = (int)((intoHour / halHour) * 60.0);
    String minStr = (minNum == 0) ? (languageEnglish ? String("0") : String("א")) : hebrewNumeral(minNum);
    return hebrewNumeral(hourNum) + ":" + minStr;
  }
}

void drawZmanimList() {
  display.fillScreen(GxEPD_WHITE);

  printBilingualRight("זמנים", "ZMANIM", 188, 26, frankruhl_hebrew_18, u8g2_font_helvB12_tf);
  // Month now shown alongside the day, per Andrew's request -- was
  // previously just the day number/gematria alone.
  if (!languageEnglish) {
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    String dateStr = String(heDay) + " " + String(heMonth);
    printHebrewLeft(dateStr.c_str(), 12, 26);
  } else {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    char dayBuf[4];
    snprintf(dayBuf, sizeof(dayBuf), "%d", heDayNum);
    String dateStr = String(dayBuf) + " " + String(heMonthEn);
    printHebrewLeft(dateStr.c_str(), 12, 26); // safe: printHebrewLeft skips RTL reversal in English mode
  }

  display.drawLine(8, 36, 192, 36, GxEPD_BLACK);

  struct ZmanRow { const char* he; const char* en; time_t epoch; };
  ZmanRow rows[] = {
    { "עלות השחר",     "Dawn",           alotEpoch },
    { "נץ החמה",        "Sunrise",       sunriseEpoch },
    { "סוף זמן ק\"ש",   "Latest Shema",  zmanimMethodMGA ? sofZmanShmaMGAEpoch : sofZmanShmaEpoch },
    { "חצות",           "Midday",        chatzotEpoch },
    { "מנחה גדולה",     "Earliest Mincha", minchaGedolaEpoch },
    { "שקיעה",          "Sunset",        sunsetEpoch },
    { "צאת הכוכבים",    "Nightfall",     tzeitEpoch },
  };
  const int rowCount = sizeof(rows) / sizeof(rows[0]);
  int y = 56;
  for (int i = 0; i < rowCount; i++) {
    printBilingualRight(rows[i].he, rows[i].en, 188, y, frankruhl_hebrew_12, u8g2_font_5x7_tf);
    // CHANGE 2026-07-13: English mode now shows ordinary WALL-CLOCK
    // time (the real Hebcal-computed time-of-day for each zman, e.g.
    // Sunrise really is 5:58 AM) rather than halachic-hour notation --
    // per Andrew's request. No new data/fetch needed: sunriseEpoch and
    // every other epoch here already come straight from Hebcal's own
    // ISO8601 timestamps (see fetchZmanim()/parseISOToEpoch()), so
    // epochToWallClockTime() just formats that same already-correct
    // epoch as ordinary local time. Hebrew mode keeps the halachic-hour
    // notation via epochToHalachicTime(), unchanged.
    String timeStr = languageEnglish ? epochToWallClockTime(rows[i].epoch) : epochToHalachicTime(rows[i].epoch);
    u8g2Fonts.setFont(languageEnglish ? u8g2_font_5x7_tf : frankruhl_hebrew_12);
    printHebrewLeft(timeStr.c_str(), 12, y);
    y += 20;
    if (i < rowCount - 1) {
      for (int xx = 12; xx < 188; xx += 5) {
        display.drawPixel(xx, y - 13, GxEPD_BLACK);
      }
    }
  }
}

// ============================================================
// Screen 4 (case 3) — Calendar Info (Omer counter / next holiday)
// MODIFIED 2026-07-13:
//   1. REAL BUG FIX (both languages): the Omer hero-day number was
//      always rendered with frankruhl_hebrew_18, which has no digit
//      glyphs -- so this number likely rendered as blank/missing glyphs
//      in BOTH Hebrew and English mode, since dayBuf is always plain
//      Arabic numerals regardless of language. Fixed by using a
//      digit-capable font for this specific number.
//   2. Holiday name (nextHolidayHebrew) was Hebrew-only with no English
//      equivalent -- now branches to nextHolidayEn (fetched from
//      Hebcal's own "title" field) in English mode.
// ============================================================
void drawCalendarInfo(time_t nowUTC) {
  display.fillScreen(GxEPD_WHITE);

  if (omerDay > 0) {
    printBilingualCentered("ספירת העומר", "OMER COUNT", 100, 26, frankruhl_hebrew_18, u8g2_font_helvB12_tf);

    char dayBuf[4];
    snprintf(dayBuf, sizeof(dayBuf), "%d", omerDay);
    u8g2Fonts.setFont(u8g2_font_helvB18_tf); // digit-capable stock font -- frankruhl_hebrew_18 has no ASCII digits
    printPlainCentered(dayBuf, 100, 76);

    String captionHe = String("יום ") + hebrewNumeral(omerDay) + " בעומר";
    String captionEn = String("Day ") + String(omerDay) + " of the Omer";
    printBilingualCentered(captionHe.c_str(), captionEn.c_str(), 100, 96, frankruhl_hebrew_12, u8g2_font_5x7_tf);

    int barX = 24, barY = 104, barW = 152, barH = 7;
    display.drawRect(barX, barY, barW, barH, GxEPD_BLACK);
    int fillW = (int)((barW - 2) * (omerDay / 49.0));
    for (int yy = barY + 1; yy < barY + barH - 1; yy += 3) {
      for (int xx = barX + 1; xx < barX + 1 + fillW; xx += 3) {
        display.drawPixel(xx, yy, GxEPD_BLACK);
      }
    }
  } else if (haveNextHoliday) {
    // Renamed from "NEXT HOLIDAY"/"החג הבא" to "HOLIDAYS"/"חגים" per
    // Andrew's request.
    printBilingualCentered("חגים", "HOLIDAYS", 100, 26, frankruhl_hebrew_18, u8g2_font_helvB12_tf);
    display.drawLine(8, 36, 192, 36, GxEPD_BLACK);

    if (languageEnglish) {
      u8g2Fonts.setFont(u8g2_font_helvB12_tf);
      printPlainCentered(nextHolidayEn, 100, 68);
    } else {
      u8g2Fonts.setFont(frankruhl_hebrew_18);
      printHebrewCentered(nextHolidayHebrew, 100, 68);
    }

    int days = daysUntilNextHoliday(nowUTC);
    String captionHe, captionEn;
    if (days <= 0) { captionHe = "היום"; captionEn = "Today"; }
    else if (days == 1) { captionHe = "מחר"; captionEn = "Tomorrow"; }
    else {
      captionHe = String("בעוד ") + hebrewNumeral(days) + " ימים";
      captionEn = String("in ") + String(days) + " days";
    }
    printBilingualCentered(captionHe.c_str(), captionEn.c_str(), 100, 90, frankruhl_hebrew_12, u8g2_font_5x7_tf);

    display.drawLine(8, 108, 192, 108, GxEPD_BLACK);
    printBilingualCentered("הדלקת נרות הבאה", "NEXT CANDLE LIGHTING", 100, 124,
                            frankruhl_hebrew_12, u8g2_font_5x7_tf);
    if (haveNextCandle) {
      // CHANGE 2026-07-13: English mode now shows ordinary WALL-CLOCK
      // time for candle lighting, per Andrew's request -- this also
      // removes the day-length approximation that affected the
      // halachic-hour version (epochToHalachicTime() uses TODAY's
      // cached sunriseEpoch/sunsetEpoch to compute halachic hour
      // length, which is a small approximation for a FUTURE day's
      // candle-lighting time; wall-clock time has no such dependency,
      // since it's just reading the real UTC epoch's local hour/minute
      // directly). Hebrew mode keeps the halachic-hour notation via
      // epochToHalachicTime(), where that approximation still applies
      // and remains worth flagging for that language only.
      String timeStr = languageEnglish ? epochToWallClockTime(nextCandleEpoch) : epochToHalachicTime(nextCandleEpoch);
      int candleDays = (int)floor((double)(nextCandleEpoch - nowUTC) / 86400.0);
      String dayLabelHe, dayLabelEn;
      if (candleDays <= 0) { dayLabelHe = "היום"; dayLabelEn = "today"; }
      else if (candleDays == 1) { dayLabelHe = "מחר"; dayLabelEn = "tomorrow"; }
      else { dayLabelHe = String("בעוד ") + hebrewNumeral(candleDays) + " ימים"; dayLabelEn = String("in ") + String(candleDays) + " days"; }

      // Occasion line. BUG FIX 2026-07-13 (part 1): previously ALWAYS
      // shown in English regardless of language mode -- fixed so the
      // Hebrew-mode page is fully Hebrew, using the same parasha name
      // table as the Torah Quotes screen for regular Shabbatot.
      //
      // CHANGE 2026-07-13 (part 2), per Andrew's explicit request: when
      // the candle-lighting memo names a regular parasha ("Parashat
      // X"), this line now simply reads "Shabbat" / "שבת" instead of
      // naming the specific parasha -- since a parasha-named candle
      // lighting always means an ordinary Shabbat (as opposed to a
      // holiday eve, e.g. "Shavuot I", which still names the holiday).
      // This applies identically in both languages.
      if (nextCandleMemo[0] != '\0') {
        String memo = String(nextCandleMemo);
        const char* prefix = "Parashat ";
        if (memo.startsWith(prefix)) {
          // Regular Shabbat -- show "Shabbat"/"שבת" rather than the
          // specific parasha name.
          if (languageEnglish) {
            u8g2Fonts.setFont(u8g2_font_5x7_tf);
            printPlainCentered("Shabbat", 100, 140);
          } else {
            u8g2Fonts.setFont(frankruhl_hebrew_12);
            printHebrewCentered("שבת", 100, 140);
          }
        } else {
          // Holiday-eve memo (e.g. "Shavuot I") -- still names the
          // specific holiday, since "Shabbat" wouldn't be accurate
          // here. No Hebrew lookup table exists for holiday-eve names
          // yet, so Hebrew mode still falls back to English text for
          // this specific case -- a smaller, clearly-scoped remaining
          // gap distinct from the regular-Shabbat case above.
          String occasionEn = String("for ") + memo;
          u8g2Fonts.setFont(u8g2_font_5x7_tf);
          printPlainCentered(occasionEn.c_str(), 100, 140);
        }
      }

      u8g2Fonts.setFont(languageEnglish ? u8g2_font_helvB12_tf : frankruhl_hebrew_18);
      printPlainCentered(timeStr.c_str(), 100, 164);
      printBilingualCentered(dayLabelHe.c_str(), dayLabelEn.c_str(), 100, 184, frankruhl_hebrew_12, u8g2_font_5x7_tf);
    } else {
      printBilingualCentered("אין נתונים", "no data", 100, 152, frankruhl_hebrew_12, u8g2_font_5x7_tf);
    }
  } else {
    printBilingualCentered("לוח שנה", "CALENDAR", 100, 26, frankruhl_hebrew_18, u8g2_font_helvB12_tf);
    display.drawLine(8, 36, 192, 36, GxEPD_BLACK);
    printBilingualCentered("אין נתוני חג", "No holiday data", 100, 100, frankruhl_hebrew_12, u8g2_font_5x7_tf);
  }
}

// ============================================================
// Screen 5 (case 4) — Torah Quotes
// REPLACES the honest stub (drawTorahQuotesStub). Real implementation:
//   - header shows this week's parasha name, bilingual (Hebrew name
//     fetched via fetchNextHoliday()'s new parashat-category capture;
//     English name is whatever bare string Hebcal itself returned,
//     stripped of "Parashat " -- no separate name-translation table)
//   - body shows today's single aliyah quote (looked up from the
//     LittleFS-cached CSV via findTodayQuote(), keyed on
//     torahParashaName + today's 1-7 day-of-week)
//   - auto-shrinking word-wrap: tries each font size from largest to
//     smallest until the wrapped text fits the available vertical
//     band, so quotes of very different lengths all render legibly
//     without any manual per-quote tuning
//   - day-of-week indicator in the bottom-right corner, bilingual
//   - honest empty state if no quote is found for today's
//     parasha+day combination (e.g. not yet filled in)
// ============================================================
struct FontLadderEntry { const uint8_t* font; int lineHeight; };
// Hebrew ladder: largest to smallest, from the confirmed available set
// in shaon_fonts.h. English ladder uses the stock U8g2 fonts already
// linked into this build.
static const FontLadderEntry HEBREW_FONT_LADDER[] = {
  { frankruhl_hebrew_18, 20 },
  { frankruhl_hebrew_12v2, 15 },
  { frankruhl_hebrew_11, 14 },
  { frankruhl_hebrew_10, 13 },
};
static const FontLadderEntry ENGLISH_FONT_LADDER[] = {
  { u8g2_font_helvB12_tf, 16 },
  { u8g2_font_5x7_tf, 10 },
};

// Word-wrap a string at word boundaries to fit within maxWidth pixels
// at the currently-set font. Returns the wrapped lines.
static std::vector<String> wrapTextToWidth(const String& text, int maxWidth) {
  std::vector<String> lines;
  int start = 0;
  int len = text.length();
  while (start < len) {
    int lastGoodBreak = -1;
    int end = start;
    // Find the furthest word-boundary break that still fits.
    for (int i = start; i <= len; i++) {
      if (i == len || text[i] == ' ') {
        String candidate = text.substring(start, i);
        int w = u8g2Fonts.getUTF8Width(candidate.c_str());
        if (w <= maxWidth) {
          lastGoodBreak = i;
          end = i;
        } else {
          break;
        }
        if (i == len) break;
      }
    }
    if (lastGoodBreak == -1) {
      // Single word wider than maxWidth -- take it anyway rather than
      // looping forever; this is a rare edge case for curated content.
      int nextSpace = text.indexOf(' ', start);
      end = (nextSpace == -1) ? len : nextSpace;
    }
    lines.push_back(text.substring(start, end));
    start = end + 1; // skip the space
    while (start < len && text[start] == ' ') start++;
  }
  return lines;
}

void drawTorahQuotes(time_t nowUTC) {
  display.fillScreen(GxEPD_WHITE);

  // Header: this week's parasha name, bilingual.
  if (haveTorahParasha) {
    if (languageEnglish) {
      u8g2Fonts.setFont(u8g2_font_helvB12_tf);
      String upper = String(torahParashaName);
      upper.toUpperCase();
      printPlainCentered(upper.c_str(), 100, 26);
    } else {
      // BUG FIX 2026-07-13: previously showed the bare ENGLISH parasha
      // name here (torahParashaName IS the CSV lookup key, e.g.
      // "Devarim" -- it was never meant to double as a Hebrew display
      // string), meaning the Hebrew-mode Torah Quotes page had an
      // English word sitting in its header, breaking the intended
      // all-Hebrew page. Now uses parshaNameToHebrew() to look up the
      // real Hebrew name (e.g. "דְּבָרִים") from the same 61-entry
      // table used for the Calendar Info candle-lighting occasion line.
      u8g2Fonts.setFont(frankruhl_hebrew_18);
      String heName = parshaNameToHebrew(String(torahParashaName));
      printHebrewCentered(heName.c_str(), 100, 26); // goes through the same RTL reversal as every other Hebrew string in this file
    }
  } else {
    printBilingualCentered("דברי תורה", "TORAH QUOTES", 100, 26, frankruhl_hebrew_18, u8g2_font_helvB12_tf);
  }
  display.drawLine(8, 36, 192, 36, GxEPD_BLACK);

  int todayDay = computeTodayDayOfWeek(nowUTC);
  String quoteHe, quoteEn;
  bool found = haveTorahParasha &&
               findTodayQuote(String(torahParashaName), todayDay, quoteHe, quoteEn);

  if (!found) {
    printBilingualCentered("טרם נטען ציטוט ליום זה", "No quote loaded for today yet", 100, 110,
                            frankruhl_hebrew_12, u8g2_font_5x7_tf);
  } else {
    const String& quoteText = languageEnglish ? quoteEn : quoteHe;
    const FontLadderEntry* ladder = languageEnglish ? ENGLISH_FONT_LADDER : HEBREW_FONT_LADDER;
    int ladderSize = languageEnglish
      ? sizeof(ENGLISH_FONT_LADDER)/sizeof(ENGLISH_FONT_LADDER[0])
      : sizeof(HEBREW_FONT_LADDER)/sizeof(HEBREW_FONT_LADDER[0]);

    const int bodyTop = 44, bodyBottom = 192, maxTextWidth = 172; // 8px margin each side of 200px screen minus corner brackets
    std::vector<String> bestLines;
    int bestLineHeight = ladder[ladderSize-1].lineHeight;

    for (int fi = 0; fi < ladderSize; fi++) {
      u8g2Fonts.setFont(ladder[fi].font);
      // For Hebrew, wrap the LOGICAL (non-reversed) string -- reversal
      // happens per-line at draw time via printHebrewCentered, matching
      // every other Hebrew-rendering path in this file.
      std::vector<String> lines = wrapTextToWidth(quoteText, maxTextWidth);
      int totalHeight = lines.size() * ladder[fi].lineHeight;
      if (totalHeight <= (bodyBottom - bodyTop) || fi == ladderSize - 1) {
        bestLines = lines;
        bestLineHeight = ladder[fi].lineHeight;
        break;
      }
    }

    int totalHeight = bestLines.size() * bestLineHeight;
    int y = bodyTop + ((bodyBottom - bodyTop) - totalHeight) / 2 + bestLineHeight;
    for (auto& line : bestLines) {
      if (languageEnglish) {
        printPlainCentered(line.c_str(), 100, y);
      } else {
        printHebrewCentered(line.c_str(), 100, y);
      }
      y += bestLineHeight;
    }
  }

  // Day-of-week indicator, bottom-right corner, bilingual.
  display.drawLine(8, 196, 192, 196, GxEPD_BLACK);
  static const char* HEBREW_DAY_LABEL[8] = {"", "יום א׳","יום ב׳","יום ג׳","יום ד׳","יום ה׳","יום ו׳","שבת"};
  static const char* ENGLISH_DAY_LABEL[8] = {"", "Day 1","Day 2","Day 3","Day 4","Day 5","Day 6","Day 7"};
  if (todayDay >= 1 && todayDay <= 7) {
    printBilingualRight(HEBREW_DAY_LABEL[todayDay], ENGLISH_DAY_LABEL[todayDay], 188, 210,
                         frankruhl_hebrew_12, u8g2_font_5x7_tf);
  }
}

// ============================================================
// Screen 6 (case 5) — Weather
// MODIFIED 2026-07-13: humidity, wind, and "6-DAY FORECAST" labels were
// hardcoded English strings with no language branch at all -- now
// bilingual. Main header and weekday abbreviations were already
// correctly bilingual (unchanged).
// ============================================================
static const char* HEBREW_WEEKDAY[7] = {"א׳","ב׳","ג׳","ד׳","ה׳","ו׳","שבת"};
static const char* ENGLISH_WEEKDAY[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

void drawWeather() {
  display.fillScreen(GxEPD_WHITE);
  printBilingualCentered("מזג אוויר", "WEATHER", 100, 24, frankruhl_hebrew_18, u8g2_font_helvB12_tf);
  display.drawLine(8, 36, 192, 36, GxEPD_BLACK);

  if (!haveWeather) {
    printBilingualCentered("אין נתונים", "No data yet", 100, 100, frankruhl_hebrew_12, u8g2_font_5x7_tf);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, 190);
    u8g2Fonts.print("Weather fetch pending/failed");
    return;
  }

  WeatherIconCat cat = weatherCodeToIcon(weatherCode);
  drawWeatherIcon(cat, 44, 70, 22);

  char tempBuf[8];
  snprintf(tempBuf, sizeof(tempBuf), "%d\xB0", (int)round(weatherTemp));
  u8g2Fonts.setFont(u8g2_font_helvB18_tf);
  printPlainCentered(tempBuf, 118, 78);

  char humNum[8], windNum[24];
  snprintf(humNum, sizeof(humNum), "%d%%", weatherHumidity);
  snprintf(windNum, sizeof(windNum), "%d %s", (int)round(weatherWindSpeed), compassDirection(weatherWindDir));
  if (languageEnglish) {
    char humBuf[16], windBuf[28];
    snprintf(humBuf, sizeof(humBuf), "HUMIDITY %s", humNum);
    snprintf(windBuf, sizeof(windBuf), "WIND %s", windNum);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, 108); u8g2Fonts.print(humBuf);
    u8g2Fonts.setCursor(110, 108); u8g2Fonts.print(windBuf);
  } else {
    // REBUILT 2026-07-13 per Andrew's feedback: the numeric value now
    // sits to the LEFT of its Hebrew label (e.g. "73% לחות" reading
    // left-to-right), matching natural RTL flow -- previously the
    // value was placed to the RIGHT of the label (i.e., after it in
    // left-to-right terms), which read backwards. The whole combined
    // line (humidity pair + gap + wind pair) is now CENTERED on the
    // screen as one unit, rather than left-anchored starting at x=12.
    //
    // Layout built by measuring every piece's real width via
    // getUTF8Width() first, then computing a horizontal center offset
    // for the whole assembly -- avoids the earlier bug class of
    // guessing fixed pixel offsets that were never actually measured.
    // ADJUSTMENT 2026-07-13: pairGap increased from 3px to 8px per
    // Andrew's feedback that the numeric value and its Hebrew label
    // were too close together.
    const int pairGap = 8;     // px between a value and its own label
    const int groupGap = 20;   // px between the humidity pair and the wind pair
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    String humLabel = reverseUTF8("לחות");
    String windLabel = reverseUTF8("רוח");
    int humLabelW = u8g2Fonts.getUTF8Width(humLabel.c_str());
    int windLabelW = u8g2Fonts.getUTF8Width(windLabel.c_str());

    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    int humNumW = u8g2Fonts.getUTF8Width(humNum);
    int windNumW = u8g2Fonts.getUTF8Width(windNum);

    int humPairW = humNumW + pairGap + humLabelW;
    int windPairW = windNumW + pairGap + windLabelW;
    int totalW = humPairW + groupGap + windPairW;
    int startX = 100 - totalW / 2; // center the whole line on the 200px-wide screen

    // Humidity pair: value, then label immediately to its right.
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(startX, 108); u8g2Fonts.print(humNum);
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    u8g2Fonts.setCursor(startX + humNumW + pairGap, 108); u8g2Fonts.print(humLabel);

    // Wind pair: value, then label immediately to its right.
    int windPairStartX = startX + humPairW + groupGap;
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(windPairStartX, 108); u8g2Fonts.print(windNum);
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    u8g2Fonts.setCursor(windPairStartX + windNumW + pairGap, 108); u8g2Fonts.print(windLabel);
  }

  // ADJUSTMENT 2026-07-13: compressed the divider above the forecast
  // section (was y=122, now y=118) to free up vertical room, per
  // Andrew's explicit suggestion to compress the section above if
  // needed. That freed-up space is applied directly to widen the gap
  // between the forecast title and the weekday row below it (was 16px,
  // now 24px), per Andrew's specific request that this gap needed more
  // room. Icon/temp rows shifted down by the same net amount to
  // preserve their own relative spacing to the weekday row -- all
  // values re-verified to still fit within the 200px screen height and
  // stay clear of each other (icon at y=172 r=9 spans 163-181, with 7px
  // clearance above the weekday label at 156 and 15px clearance before
  // the temp row at 196).
  display.drawLine(8, 118, 192, 118, GxEPD_BLACK);

  if (languageEnglish) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, 132);
    u8g2Fonts.print("6-DAY FORECAST");
  } else {
    // Right-justified in Hebrew mode, per Andrew's request -- was
    // previously left-anchored via printHebrewLeft, which looked
    // visually inconsistent with the rest of the RTL layout on this
    // screen.
    u8g2Fonts.setFont(frankruhl_hebrew_12);
    printHebrewRight("תחזית שבועית", 188, 132);
  }

  // ADJUSTMENT 2026-07-13: all four Y positions shifted down by 4px as
  // part of freeing up room for a wider title-to-weekday gap above (see
  // the divider/title Y changes just above) -- the icon stays centered
  // in the visual gap between the weekday label's bottom and the
  // temperature's top, just at the new shifted positions.
  int weekdayY = 156;
  int iconY = 172;
  int iconR = 9;
  int tempY = 196;

  int colW = (192 - 12) / FORECAST_DAYS;
  for (int i = 0; i < FORECAST_DAYS; i++) {
    int cx = 12 + colW * i + colW / 2;
    if (forecastWeekday[i] >= 0 && forecastWeekday[i] < 7) {
      if (languageEnglish) {
        u8g2Fonts.setFont(u8g2_font_5x7_tf);
        printPlainCentered(ENGLISH_WEEKDAY[forecastWeekday[i]], cx, weekdayY);
      } else {
        u8g2Fonts.setFont(frankruhl_hebrew_12);
        printHebrewCentered(HEBREW_WEEKDAY[forecastWeekday[i]], cx, weekdayY);
      }
    }
    if (forecastCode[i] >= 0) {
      drawWeatherIcon(weatherCodeToIcon(forecastCode[i]), cx, iconY, iconR);
    }
    char t[6]; snprintf(t, sizeof(t), "%d\xB0", forecastTempMax[i]);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    int tw = u8g2Fonts.getUTF8Width(t);
    u8g2Fonts.setCursor(cx - tw/2, tempY);
    u8g2Fonts.print(t);
    if (i < FORECAST_DAYS - 1) {
      // Column divider span updated to match the shifted layout above
      // (title/weekday/icon/temp all moved as part of this round's
      // spacing fix) -- same relative offsets from the title/temp rows
      // as before (6px below title, 6px above temp).
      display.drawLine(12 + colW * (i+1), 138, 12 + colW * (i+1), 190, GxEPD_BLACK);
    }
  }
}

// English screen names for the toggle list (Settings screen).
// RENAMED 2026-07-13 per Andrew's request: "Halachic Face" -> "Digital
// Watch Face", "Halachic Analog" -> "Analog Watch Face".
static const char* SCREEN_NAMES[6] = {
  "Digital Watch Face", "Analog Watch Face", "Zmanim List",
  "Calendar Info", "Torah Quotes", "Weather"
};
// ---- NEW: Hebrew screen names, parallel to SCREEN_NAMES above ----
// Previously this array didn't exist at all -- the Settings screen's
// per-screen toggle list was English-only regardless of language mode.
static const char* SCREEN_NAMES_HE[6] = {
  "פני שעון הלכתי", "שעון הלכתי אנלוגי", "רשימת זמנים",
  "מידע לוח שנה", "דברי תורה", "מזג אוויר"
};

static const int SETTINGS_CONTENT_TOP = 36;
static const int SETTINGS_CONTENT_BOTTOM = 184;
static const int SETTINGS_FOOTER_Y = 195;

void drawProvisioningScreen(bool waitingForSubmit) {
  display.fillScreen(GxEPD_WHITE);
  u8g2Fonts.setFont(u8g2_font_helvB12_tf);
  printPlainCentered("WiFi / Location Setup", 100, 30);
  display.drawLine(8, 40, 192, 40, GxEPD_BLACK);

  u8g2Fonts.setFont(u8g2_font_5x7_tf);
  u8g2Fonts.setCursor(12, 60);  u8g2Fonts.print("1. On your phone, connect to WiFi:");
  u8g2Fonts.setFont(u8g2_font_helvB12_tf);
  printPlainCentered(SETUP_AP_SSID, 100, 78);

  u8g2Fonts.setFont(u8g2_font_5x7_tf);
  u8g2Fonts.setCursor(12, 100); u8g2Fonts.print("2. A setup page should open");
  u8g2Fonts.setCursor(12, 112); u8g2Fonts.print("   automatically. If not, go to:");
  u8g2Fonts.setFont(u8g2_font_helvB12_tf);
  printPlainCentered("192.168.4.1", 100, 130);

  u8g2Fonts.setFont(u8g2_font_5x7_tf);
  u8g2Fonts.setCursor(12, 150); u8g2Fonts.print("3. Enter WiFi name/password and");
  u8g2Fonts.setCursor(12, 162); u8g2Fonts.print("   zip code, then tap Save.");

  display.drawLine(8, 176, 192, 176, GxEPD_BLACK);
  u8g2Fonts.setCursor(12, 192);
  u8g2Fonts.print(waitingForSubmit ? "Waiting... (times out in 5 min)" : "Done.");
}

// ============================================================
// Screen 7 (case 6) — Settings
// NOT translated this session, by deliberate decision: this screen
// packs 11 rows plus 5 section headers into a tightly-scrolled 148px
// vertical band (SETTINGS_CONTENT_TOP..SETTINGS_CONTENT_BOTTOM), all
// currently sized around English text at u8g2_font_5x7_tf. The
// available Hebrew fonts run from frankruhl_hebrew_10 up to _18 in
// fixed steps -- there's no size in that ladder small enough to match
// 5x7's density without falling below the point where individual
// letterforms were already confirmed (in this file's own font-build
// comments) to lose legibility. Rather than force an awkward, cramped,
// or illegible Hebrew rendering onto a screen this dense, Settings
// stays English-only for now, unchanged from the original
// implementation -- this was an explicit call, not an oversight.
// GRA/MGA are similarly left as their original Latin-letter form here,
// consistent with leaving the rest of this screen untouched. The dead
// SCREEN_NAMES_HE array above is a leftover from before this decision --
// harmless (referenced nowhere), kept in case Settings translation is
// revisited later with a different layout.
// ============================================================
void drawSettings() {
  display.fillScreen(GxEPD_WHITE);
  // Title changed to always show the English word "Settings", per
  // Andrew's explicit request -- unlike every other bilingual header in
  // this file, this one does NOT branch on languageEnglish.
  u8g2Fonts.setFont(u8g2_font_helvB12_tf);
  printPlainCentered("Settings", 100, 22);
  display.drawLine(8, 32, 192, 32, GxEPD_BLACK);

  // Layout note (2026-07-13): the WiFi/Zip row now spans TWO display
  // lines (WiFi name, then Zip code on its own line) instead of one
  // combined line, and a new "Last Updated" line was added below Serve
  // Debug Log. Both are informational display-only lines -- there is
  // nothing to toggle on a zip code or a timestamp -- so per Andrew's
  // confirmation, settingsCursor's 0-10 indexing and 11-row wraparound
  // are UNCHANGED; these two lines are extra vertical space inserted
  // into the layout, not new cursor stops. rowY[9] (WiFi/Zip Setup) and
  // rowY[10] (Serve Debug Log) still map to the same cursor values as
  // before; the new display-only lines get their own Y positions
  // computed relative to rowY[9]/rowY[10] rather than their own cursor
  // slot.
  // BUG FIX 2026-07-13: Zip is now rowY[10], a REAL cursor stop (was
  // previously just "zipRowY", a passive display-only Y position that
  // the cursor could never actually land on -- see the button-handling
  // fix in setup() for the full explanation). Serve Debug Log shifted
  // from rowY[10] to rowY[11] to make room. rowY[] grown from 11 to 12
  // slots to match.
  int rowY[12];
  int headerY[5];
  int simY = 44;
  headerY[0] = simY;            simY += 14; rowY[0] = simY;
  simY += 12; simY += 12;
  headerY[1] = simY;            simY += 14; rowY[1] = simY;
  simY += 12; simY += 12;
  headerY[2] = simY;            simY += 14; rowY[2] = simY;
  simY += 12; simY += 12;
  headerY[3] = simY;            simY += 14;
  int screenRowY[6];
  for (int i = 0; i < 6; i++) { screenRowY[i] = simY; simY += 13; }
  for (int i = 0; i < 6; i++) rowY[3 + i] = screenRowY[i];
  simY += 12; simY += 12;
  headerY[4] = simY;            simY += 14; rowY[9] = simY;   // WiFi
  simY += 13; rowY[10] = simY;                                // Zip (NOW a real cursor stop)
  simY += 13; rowY[11] = simY;                                // Serve Debug Log
  // Last Updated display line, directly below Serve Debug Log -- still
  // a passive display-only line (nothing to toggle on a timestamp), so
  // it correctly stays outside the cursor's addressable rows.
  simY += 12; int lastUpdatedRowY = simY;
  simY += 12; // room for the section-divider line under Setup
  int totalContentBottom = simY;

  int boxTopMargin = (settingsCursor == 0) ? 12 : 9;
  int selectedY = rowY[settingsCursor];
  int scrollOffset = 0;
  if (selectedY + 3 > SETTINGS_CONTENT_BOTTOM) {
    scrollOffset = (selectedY + 3) - SETTINGS_CONTENT_BOTTOM;
  } else if (selectedY - boxTopMargin < SETTINGS_CONTENT_TOP) {
    scrollOffset = (selectedY - boxTopMargin) - SETTINGS_CONTENT_TOP;
  }
  int maxScroll = totalContentBottom - SETTINGS_CONTENT_BOTTOM;
  if (maxScroll < 0) maxScroll = 0;
  if (scrollOffset > maxScroll) scrollOffset = maxScroll;
  if (scrollOffset < 0) scrollOffset = 0;

  auto visible = [&](int yPos) -> bool {
    int scrolledY = yPos - scrollOffset;
    return scrolledY >= SETTINGS_CONTENT_TOP - 2 && scrolledY <= SETTINGS_CONTENT_BOTTOM + 2;
  };

  int y;

  // Row 0: Language (Hebrew / English)
  if (visible(headerY[0])) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, headerY[0] - scrollOffset);
    u8g2Fonts.print("LANGUAGE");
  }
  y = rowY[0] - scrollOffset;
  if (visible(rowY[0])) {
    bool selected = (settingsCursor == 0);
    if (selected) display.drawRect(8, y - 12, 184, 15, GxEPD_BLACK);
    display.drawCircle(24, y - 3, 4, GxEPD_BLACK);
    if (!languageEnglish) display.fillCircle(24, y - 3, 2, GxEPD_BLACK);
    // These two labels are intentionally NOT translated by the language
    // toggle -- they name the two options themselves (so the person can
    // always find their way back regardless of current selection), not
    // content that changes meaning with language mode.
    u8g2Fonts.setFont(frankruhl_hebrew_12v2);
    printHebrewLeft("עברית", 34, y);
    display.drawCircle(84, y - 3, 4, GxEPD_BLACK);
    if (languageEnglish) display.fillCircle(84, y - 3, 2, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(94, y); u8g2Fonts.print("English");
  }
  if (visible(rowY[0] + 12)) {
    display.drawLine(8, rowY[0] + 12 - scrollOffset, 192, rowY[0] + 12 - scrollOffset, GxEPD_BLACK);
  }

  // Row 1: Temperature unit (F / C)
  if (visible(headerY[1])) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, headerY[1] - scrollOffset);
    u8g2Fonts.print("TEMPERATURE");
  }
  y = rowY[1] - scrollOffset;
  if (visible(rowY[1])) {
    bool selected = (settingsCursor == 1);
    if (selected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    display.drawCircle(24, y - 3, 4, GxEPD_BLACK);
    if (tempFahrenheit) display.fillCircle(24, y - 3, 2, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(34, y); u8g2Fonts.print("\xB0" "F");
    display.drawCircle(84, y - 3, 4, GxEPD_BLACK);
    if (!tempFahrenheit) display.fillCircle(84, y - 3, 2, GxEPD_BLACK);
    u8g2Fonts.setCursor(94, y); u8g2Fonts.print("\xB0" "C");
  }
  if (visible(rowY[1] + 12)) {
    display.drawLine(8, rowY[1] + 12 - scrollOffset, 192, rowY[1] + 12 - scrollOffset, GxEPD_BLACK);
  }

  // Row 2: Zmanim method (GRA / MGA)
  if (visible(headerY[2])) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, headerY[2] - scrollOffset);
    u8g2Fonts.print("ZMANIM METHOD");
  }
  y = rowY[2] - scrollOffset;
  if (visible(rowY[2])) {
    bool selected = (settingsCursor == 2);
    if (selected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    display.drawCircle(24, y - 3, 4, GxEPD_BLACK);
    if (!zmanimMethodMGA) display.fillCircle(24, y - 3, 2, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(34, y); u8g2Fonts.print("GRA");
    display.drawCircle(84, y - 3, 4, GxEPD_BLACK);
    if (zmanimMethodMGA) display.fillCircle(84, y - 3, 2, GxEPD_BLACK);
    u8g2Fonts.setCursor(94, y); u8g2Fonts.print("MGA");
  }
  if (visible(rowY[2] + 12)) {
    display.drawLine(8, rowY[2] + 12 - scrollOffset, 192, rowY[2] + 12 - scrollOffset, GxEPD_BLACK);
  }

  // "SCREENS (ON/OFF)" header + 6 checkbox rows
  if (visible(headerY[3])) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, headerY[3] - scrollOffset);
    u8g2Fonts.print("SCREENS (ON/OFF)");
  }
  for (int i = 0; i < 6; i++) {
    if (!visible(screenRowY[i])) continue;
    y = screenRowY[i] - scrollOffset;
    bool selected = (settingsCursor == i + 3);
    if (selected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    display.drawRect(20, y - 8, 10, 10, GxEPD_BLACK);
    if (screenEnabled[i]) {
      display.drawLine(22, y - 3, 24, y - 1, GxEPD_BLACK);
      display.drawLine(24, y - 1, 28, y - 7, GxEPD_BLACK);
    }
    // BUG FIX 2026-07-13: this line never explicitly called setFont()
    // before printing -- it silently relied on whatever font happened
    // to already be active from earlier in this same function call
    // (u8g2_font_5x7_tf, if the SCREENS header above was also drawn
    // this frame). Once scrolled far enough that headerY[3] falls
    // outside the visible band (which happens once scrollOffset grows
    // past ~26px -- confirmed by simulating the real Y-position math),
    // that setFont() call above never runs, and this loop was drawing
    // with whatever font was ACTUALLY left over from before this
    // function -- e.g. u8g2_font_helvB12_tf from the "Settings" title
    // at the very top of this same draw call. This is exactly the "font
    // size increases strangely near the bottom" symptom reported on
    // real hardware. Fixed by setting the font explicitly, every
    // iteration, regardless of whether the section header happens to
    // be visible this frame.
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(38, y);
    u8g2Fonts.print(SCREEN_NAMES[i]);
  }

  // BUG FIX 2026-07-13: this divider was genuinely MISSING entirely --
  // unlike every other section transition in this screen (Language->
  // Temperature, Temperature->Zmanim Method, Zmanim Method->Screens all
  // have an explicit divider line drawn right after their content),
  // there was no line at all between the end of the screen-toggle list
  // BUG FIX 2026-07-13 (v2): the first attempt at this divider anchored
  // its position to screenRowY[5] + 12, which looked right by analogy
  // to the other section dividers, but was actually wrong -- it didn't
  // account for the screen-row loop's OWN internal line-height advance
  // (simY += 13 happens for EACH of the 6 rows, so by the time the loop
  // exits, simY has already moved 13px past screenRowY[5] before the
  // "+12+12" gap even begins). Confirmed via direct step-by-step
  // simulation of the real layout arithmetic: this produced a 25px gap
  // between the divider and the SETUP header, versus the 12px gap used
  // at every other section boundary -- exactly matching Andrew's report
  // of an uneven, larger gap here. Fixed by anchoring the divider
  // relative to headerY[4] instead (working backward by the same 12px
  // used everywhere else), which is correct regardless of the screen-
  // row loop's internal step size, rather than trying to re-derive the
  // same position forward from screenRowY[5].
  int screensToSetupDividerY = headerY[4] - 12;
  if (visible(screensToSetupDividerY)) {
    display.drawLine(8, screensToSetupDividerY - scrollOffset, 192, screensToSetupDividerY - scrollOffset, GxEPD_BLACK);
  }

  // "SETUP" header + WiFi (row 9), Zip (row 10), Serve Debug Log
  // (row 11) -- all THREE now independently selectable cursor stops.
  //
  // BUG FIX 2026-07-13: Andrew reported not being able to land the
  // cursor on Zip at all, and only seeing it "render after you scroll
  // past it" -- confirmed as a real structural bug: Zip was previously
  // just a passive DISPLAY line squeezed between WiFi (cursor 9) and
  // Serve Debug Log (cursor 10), with no cursor value of its own, so
  // the cursor could only ever pass through its Y position while moving
  // between the two real rows on either side of it -- it could never
  // actually stop there. Fixed by giving Zip its own real cursor slot
  // (10), with Serve Debug Log shifted to 11 to make room (see the
  // settingsCursor wraparound and hold-action handling in setup() for
  // the matching fix there). Each of the three rows now has its own
  // independent selection box and "HOLD to configure"/"HOLD to
  // activate" hint, rather than sharing one combined state.
  if (visible(headerY[4])) {
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, headerY[4] - scrollOffset);
    u8g2Fonts.print("SETUP");
  }

  // Row 9: WiFi
  if (visible(rowY[9])) {
    y = rowY[9] - scrollOffset;
    bool wifiSelected = (settingsCursor == 9);
    if (wifiSelected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, y);
    u8g2Fonts.print("WiFi: ");
    if (wifiSsid[0] != '\0') {
      u8g2Fonts.print(wifiSsid);
    } else {
      u8g2Fonts.print("not set");
    }
    if (wifiSelected) {
      const char* hint = "HOLD to configure";
      int hintW = u8g2Fonts.getUTF8Width(hint);
      u8g2Fonts.setCursor(188 - hintW, y);
      u8g2Fonts.print(hint);
    }
  }

  // Row 10: Zip -- now a REAL, independently selectable cursor stop,
  // with its own selection box and hint, exactly matching the WiFi row
  // above. Holding on this row triggers the SAME WiFi/Location setup
  // captive portal as the WiFi row (zip code is set through that same
  // form), since there's no separate zip-only entry flow.
  if (visible(rowY[10])) {
    y = rowY[10] - scrollOffset;
    bool zipSelected = (settingsCursor == 10);
    if (zipSelected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, y);
    u8g2Fonts.print("Zip: ");
    if (zipCode[0] != '\0') {
      u8g2Fonts.print(zipCode);
    } else {
      u8g2Fonts.print("not set");
    }
    if (zipSelected) {
      const char* hint = "HOLD to configure";
      int hintW = u8g2Fonts.getUTF8Width(hint);
      u8g2Fonts.setCursor(188 - hintW, y);
      u8g2Fonts.print(hint);
    }
  }

  // Row 11: Serve Debug Log (shifted from 10 to 11 to make room for Zip).
  if (visible(rowY[11])) {
    y = rowY[11] - scrollOffset;
    bool selected = (settingsCursor == 11);
    if (selected) display.drawRect(8, y - 9, 184, 12, GxEPD_BLACK);
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, y);
    u8g2Fonts.print("Serve Debug Log");
    if (selected) {
      const char* hint = "HOLD to activate";
      int hintW = u8g2Fonts.getUTF8Width(hint);
      u8g2Fonts.setCursor(188 - hintW, y);
      u8g2Fonts.print(hint);
    }
  }

  // Last Updated line -- NEW 2026-07-13, replaces the sync
  // freshness/staleness icon that used to appear on Screen 1. Shows the
  // last successful full data sync (fresh boot NTP sync, sunset
  // rollover, or manual WiFi/Zip setup re-fetch -- see
  // lastSuccessfulUpdateEpoch's three set-sites in setup()). Not a
  // cursor stop -- purely informational, matching the Zip line above.
  if (visible(lastUpdatedRowY)) {
    y = lastUpdatedRowY - scrollOffset;
    u8g2Fonts.setFont(u8g2_font_5x7_tf);
    u8g2Fonts.setCursor(12, y);
    if (lastSuccessfulUpdateEpoch > 0) {
      struct tm updTm;
      time_t shifted = lastSuccessfulUpdateEpoch + GMT_OFFSET_SEC + DST_OFFSET_SEC;
      gmtime_r(&shifted, &updTm);
      char buf[40];
      snprintf(buf, sizeof(buf), "Last Updated: %02d:%02d, %02d/%02d/%04d",
        updTm.tm_hour, updTm.tm_min, updTm.tm_mon+1, updTm.tm_mday, updTm.tm_year+1900);
      u8g2Fonts.print(buf);
    } else {
      u8g2Fonts.print("Last Updated: never");
    }
  }

  // Closing divider for the SETUP section -- previously the only
  // section without one, per Andrew's feedback that it should match
  // the visual style of every other section.
  if (visible(lastUpdatedRowY + 10)) {
    display.drawLine(8, lastUpdatedRowY + 10 - scrollOffset, 192, lastUpdatedRowY + 10 - scrollOffset, GxEPD_BLACK);
  }

  // Footer instructions -- always fixed at the very bottom, never part of
  // the scrolling content, so it can never overlap a row.
  display.drawLine(8, SETTINGS_FOOTER_Y - 9, 192, SETTINGS_FOOTER_Y - 9, GxEPD_BLACK);
  u8g2Fonts.setFont(u8g2_font_5x7_tf);
  u8g2Fonts.setCursor(12, SETTINGS_FOOTER_Y);
  u8g2Fonts.print("DOWN: move    HOLD: toggle");
}

// ---- Screen dispatch ----
void drawCurrentScreen(time_t now, const String& heroTime, int battPct, bool syncFresh, bool charging) {
  switch (currentScreen) {
    case 0:  drawScreen1(now, heroTime, battPct, syncFresh, charging); break;
    case 1:  drawHalachicAnalog(now, battPct, charging); break;
    case 2:  drawZmanimList(); break;
    case 3:  drawCalendarInfo(now); break;
    case 4:  drawTorahQuotes(now); break;
    case 5:  drawWeather(); break;
    case 6:  drawSettings(); break;
    default: drawScreen1(now, heroTime, battPct, syncFresh, charging); break;
  }
}

void dumpPartitionTable() {
  Serial.println("---- Real partition table (from device) ----");
  Preferences partDumpPrefs;
  partDumpPrefs.begin("shaon", false);
  String dump = "";
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA,
    ESP_PARTITION_SUBTYPE_ANY, NULL);
  bool foundAny = false;
  while (it != NULL) {
    const esp_partition_t* p = esp_partition_get(it);
    char line[96];
    snprintf(line, sizeof(line), "label=%-10s subtype=0x%02x offset=0x%06lx size=0x%06lx (%luKB)\n",
      p->label, p->subtype, (unsigned long)p->address,
      (unsigned long)p->size, (unsigned long)(p->size / 1024));
    Serial.print(line);
    dump += line;
    foundAny = true;
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
  if (!foundAny) {
    const char* msg = "(no data partitions found at all -- partition table itself may be wrong/missing)\n";
    Serial.println(msg);
    dump += msg;
  }
  partDumpPrefs.putString("partdump", dump);
  partDumpPrefs.end();
  Serial.println("---------------------------------------------");
  Serial.println("(also saved to NVS key 'partdump' in case this was missed live)");
}

// ---- NEW 2026-07-13: shared initial data-fetch sequence ----
// Extracted from the original freshBoot block so the SAME logic can be
// reused by the new retry-with-backoff mechanism below (see
// dataFetchRetryCount) -- previously this WiFi-connect + NTP-sync +
// full-fetch sequence only ever ran once, on fresh boot, with no way to
// retry it later if it failed (confirmed real scenario: a live serial
// log showed "WiFi failed on fresh boot", after which every data-
// dependent screen stayed blank until the NEXT sunset rollover,
// potentially many hours away). Returns true only if BOTH zmanim and
// Hebrew-date fetches succeed (matching the original haveData=true
// condition exactly) -- on success, sets haveData/lastSyncEpoch/
// lastSuccessfulUpdateEpoch exactly as the original fresh-boot code did.
// On failure, leaves those untouched so the caller's own retry-counter
// logic decides what happens next.
bool attemptInitialDataFetch() {
  Serial.println("Attempting initial data fetch (WiFi + NTP + zmanim/date/weather)");
  if (!wifiConnect()) {
    Serial.println("attemptInitialDataFetch: WiFi connect failed");
    return false;
  }
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, "pool.ntp.org");
  struct tm ti;
  bool synced = false;
  for (int attempt = 0; attempt < 20; attempt++) {
    if (getLocalTime(&ti, 1000) && (ti.tm_year + 1900) >= 2025) {
      synced = true;
      break;
    }
    delay(500);
  }
  if (!synced) {
    Serial.println("attemptInitialDataFetch: NTP sync failed");
    wifiOff();
    return false;
  }
  Serial.printf("NTP time: %04d-%02d-%02d %02d:%02d:%02d\n",
    ti.tm_year+1900,ti.tm_mon+1,ti.tm_mday,ti.tm_hour,ti.tm_min,ti.tm_sec);
  int gy=ti.tm_year+1900, gm=ti.tm_mon+1, gd=ti.tm_mday;
  bool okZ = fetchZmanim(gy,gm,gd);
  bool okD = fetchHebrewDate(gy,gm,gd);
  fetchOmerDay(gy,gm,gd);
  time_t nowForHoliday; time(&nowForHoliday);
  fetchNextHoliday(gy,gm,gd,nowForHoliday);
  fetchWeather();
  wakesSinceWeatherFetch = 0;
  moonFrac = calcMoonFraction(gy,gm,gd);
  Serial.printf("Moon fraction: %.3f\n", moonFrac);
  fetchTorahQuotesCSV();
  wifiOff();
  if (okZ && okD) {
    haveData = true;
    time(&lastSyncEpoch);
    time(&lastSuccessfulUpdateEpoch);
    return true;
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
  bool freshBoot = (wake!=ESP_SLEEP_WAKEUP_TIMER && wake!=ESP_SLEEP_WAKEUP_EXT1);

  if (freshBoot) {
    delay(3000);
  } else {
    delay(200);
  }

  {
    Preferences prevDump;
    prevDump.begin("shaon", true);
    String saved = prevDump.getString("partdump", "");
    prevDump.end();
    if (saved.length() > 0) {
      Serial.println("---- Partition table saved from a PREVIOUS boot (NVS) ----");
      Serial.print(saved);
      Serial.println("------------------------------------------------------------");
    }
  }
  dumpPartitionTable();

  ensureLittleFsMounted();

  prefs.begin("shaon", false);

  SPI.begin(SPI_SCK,SPI_MISO,SPI_MOSI,DISPLAY_CS);
  display.init(115200, freshBoot);
  u8g2Fonts.begin(display);
  u8g2Fonts.setFontMode(1);
  u8g2Fonts.setForegroundColor(GxEPD_BLACK);
  u8g2Fonts.setBackgroundColor(GxEPD_WHITE);

  bool screenChanged = false;
  bool forceRefreshOnly = false;
  if (wake == ESP_SLEEP_WAKEUP_EXT1) {
    uint64_t wakeStatus = esp_sleep_get_ext1_wakeup_status();

    // ---- BUTTON REMAPPING 2026-07-13 ----
    // Per Andrew's request: top-right (UP_BTN_PIN) and bottom-left
    // (MENU_BTN_PIN) have swapped roles. UP_BTN_PIN now cycles forward
    // through screens (previously MENU's job). MENU_BTN_PIN now does a
    // plain refresh on tap, PLUS a new hold-to-toggle-language feature
    // (previously UP was refresh-only, with no hold behavior at all).
    // BACK_BTN_PIN (top-left, cycle backward) and DOWN_BTN_PIN
    // (bottom-right, Settings-only) are both unchanged.
    if (wakeStatus & (1ULL << UP_BTN_PIN)) {
      int next = currentScreen;
      for (int tries = 0; tries < NUM_SCREENS; tries++) {
        next = (next + 1) % NUM_SCREENS;
        if (next == 6 || screenEnabled[next]) break;
      }
      currentScreen = next;
      screenChanged = true;
      Serial.printf("UP (top-right) pressed -> cycle forward -> screen %d\n", currentScreen);
    }

    if (wakeStatus & (1ULL << BACK_BTN_PIN)) {
      int prev = currentScreen;
      for (int tries = 0; tries < NUM_SCREENS; tries++) {
        prev = (prev - 1 + NUM_SCREENS) % NUM_SCREENS;
        if (prev == 6 || screenEnabled[prev]) break;
      }
      currentScreen = prev;
      screenChanged = true;
      Serial.printf("BACK (top-left) pressed -> cycle backward -> screen %d\n", currentScreen);
    }

    if (wakeStatus & (1ULL << MENU_BTN_PIN)) {
      // Reuses the exact hold-detection pattern already established
      // for DOWN_BTN_PIN below: the wake event itself is edge-
      // triggered (fires the instant the pin goes low), so detecting a
      // hold means staying awake briefly and polling the pin directly.
      // MENU_BTN_PIN is active-low (same ESP_EXT1_WAKEUP_ANY_LOW
      // convention as every other button here), so "still pressed"
      // means the pin reads LOW.
      pinMode(MENU_BTN_PIN, INPUT_PULLUP);
      const unsigned long MENU_HOLD_THRESHOLD_MS = 600;
      unsigned long menuPressStart = millis();
      while (digitalRead(MENU_BTN_PIN) == LOW &&
             (millis() - menuPressStart) < MENU_HOLD_THRESHOLD_MS) {
        delay(20);
      }
      bool menuWasHold = (millis() - menuPressStart) >= MENU_HOLD_THRESHOLD_MS;

      if (menuWasHold) {
        // Global language toggle -- works from any screen, not just
        // Settings, per Andrew's explicit choice. Mirrors exactly what
        // Settings row 0's hold-to-toggle already does (same prefs key,
        // "lang"), so the two toggle paths stay in sync with each
        // other and with NVS.
        languageEnglish = !languageEnglish;
        prefs.putBool("lang", languageEnglish);
        screenChanged = true; // force a full redraw so the language change is visible immediately
        Serial.printf("MENU (bottom-left) held -> language toggled to %s\n", languageEnglish ? "English" : "Hebrew");
      } else {
        forceRefreshOnly = true;
        Serial.println("MENU (bottom-left) tapped -> forcing screen refresh");
      }
    }

    if (wakeStatus & (1ULL << DOWN_BTN_PIN)) {
      pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
      const unsigned long HOLD_THRESHOLD_MS = 600;
      unsigned long pressStart = millis();
      while (digitalRead(DOWN_BTN_PIN) == LOW &&
             (millis() - pressStart) < HOLD_THRESHOLD_MS) {
        delay(20);
      }
      bool wasHold = (millis() - pressStart) >= HOLD_THRESHOLD_MS;

      if (currentScreen == 6) {
        if (wasHold) {
          Serial.printf("DOWN held -> action on settings row %d\n", settingsCursor);
          if (settingsCursor == 0) {
            languageEnglish = !languageEnglish;
            prefs.putBool("lang", languageEnglish);
          } else if (settingsCursor == 1) {
            tempFahrenheit = !tempFahrenheit;
            prefs.putBool("tempF", tempFahrenheit);
            if (wifiConnect()) {
              fetchWeather();
              wifiOff();
            }
            wakesSinceWeatherFetch = 0;
          } else if (settingsCursor == 2) {
            zmanimMethodMGA = !zmanimMethodMGA;
            prefs.putBool("mga", zmanimMethodMGA);
          } else if (settingsCursor >= 3 && settingsCursor <= 8) {
            int idx = settingsCursor - 3;
            screenEnabled[idx] = !screenEnabled[idx];
            char key[8]; snprintf(key, sizeof(key), "scr%d", idx);
            prefs.putBool(key, screenEnabled[idx]);
          } else if (settingsCursor == 9 || settingsCursor == 10) {
            // REAL BUG FIX 2026-07-13: Zip is now its OWN real cursor
            // stop (10), not just a display line squeezed between WiFi
            // (9) and Serve Debug Log -- previously the cursor could
            // only ever be 9 or 10 (old Serve Debug Log slot), meaning
            // it was IMPOSSIBLE to land the cursor on the Zip row at
            // all; you could only scroll PAST it while moving between
            // WiFi and Serve Debug Log, exactly matching Andrew's
            // report ("it only renders after you scroll past it").
            // Both WiFi (9) and Zip (10) now trigger the same
            // WiFi/Location setup captive portal, since zip code is set
            // through that same form -- holding down on either row
            // launches identical setup.
            startProvisioning();
            if (wifiConnect()) {
              time_t nowForFetch; time(&nowForFetch);
              struct tm nt; localtime_r(&nowForFetch, &nt);
              int gy = nt.tm_year+1900, gm = nt.tm_mon+1, gd = nt.tm_mday;
              fetchZmanim(gy,gm,gd);
              fetchHebrewDate(gy,gm,gd);
              fetchOmerDay(gy,gm,gd);
              if (omerDay == 0) fetchNextHoliday(gy,gm,gd,nowForFetch);
              fetchWeather();
              wakesSinceWeatherFetch = 0;
              moonFrac = calcMoonFraction(gy,gm,gd);
              haveData = true;
              time(&lastSyncEpoch);
              time(&lastSuccessfulUpdateEpoch);
              wifiOff();
            }
          } else if (settingsCursor == 11) {
            // Serve Debug Log, shifted from cursor 10 to 11 to make
            // room for Zip's new real cursor slot at 10.
            startLogServer();
          }
        } else {
          // Wraparound updated from %11 to %12 -- one more real cursor
          // stop now that Zip is addressable (12 total: 0-11).
          settingsCursor = (settingsCursor + 1) % 12;
          Serial.printf("DOWN tapped -> settings cursor %d\n", settingsCursor);
        }
        screenChanged = true;
      }
    }
  }

  if (freshBoot) {
    zmanimMethodMGA = prefs.getBool("mga", false);
    tempFahrenheit = prefs.getBool("tempF", true);
    languageEnglish = prefs.getBool("lang", false);
    for (int i = 0; i < 6; i++) {
      char key[8]; snprintf(key, sizeof(key), "scr%d", i);
      screenEnabled[i] = prefs.getBool(key, true);
    }
    String savedSsid = prefs.getString("wifiSsid", wifiSsid);
    String savedPass = prefs.getString("wifiPass", wifiPassword);
    String savedZip  = prefs.getString("zipCode", zipCode);
    savedSsid.toCharArray(wifiSsid, sizeof(wifiSsid));
    savedPass.toCharArray(wifiPassword, sizeof(wifiPassword));
    savedZip.toCharArray(zipCode, sizeof(zipCode));
    Serial.println("Fresh boot: NTP sync + initial data fetch");
    // REFACTORED 2026-07-13: now calls the shared
    // attemptInitialDataFetch() function (see its definition above
    // setup()) instead of inline-duplicating the same WiFi/NTP/fetch
    // logic -- this same function is also called from the new retry-
    // with-backoff block further down, so both paths stay in sync
    // rather than risking drift between two copies of the same logic.
    bool fetchOk = attemptInitialDataFetch();
    if (fetchOk) {
      // Success on fresh boot: reset the retry counter to 0, so if a
      // LATER failure happens (e.g. a sunset-rollover WiFi blip), the
      // new retry mechanism starts its fast-retry phase fresh rather
      // than inheriting any prior count.
      dataFetchRetryCount = 0;
    } else {
      Serial.println("Fresh boot initial data fetch failed -- will retry on subsequent wakes with backoff");
      // Start the retry counter at 0; the retry block below (guarded by
      // `if (!haveData)`) will begin incrementing it from the very next
      // wake onward.
      dataFetchRetryCount = 0;
    }
  }

  time_t now; time(&now);
  struct tm nowtm; localtime_r(&now,&nowtm);
  Serial.printf("Now: %04d-%02d-%02d %02d:%02d:%02d\n",
    nowtm.tm_year+1900,nowtm.tm_mon+1,nowtm.tm_mday,nowtm.tm_hour,nowtm.tm_min,nowtm.tm_sec);

  time_t nowUTC = now;

  time_t lastSyncBefore = lastSyncEpoch;
  String logRollover = "n/a";
  String logWeather = "n/a";
  String logRetry = "n/a"; // new wake-log field for this retry mechanism

  // ---- NEW 2026-07-13: fresh-boot-failure retry with backoff ----
  // If haveData is still false on an ORDINARY (non-fresh-boot) wake --
  // meaning the initial fresh-boot fetch failed and hasn't succeeded
  // since -- retry attemptInitialDataFetch() here, with a backoff
  // schedule so a prolonged WiFi outage doesn't hammer the radio every
  // single minute forever (Andrew's explicit choice: fast retries at
  // first, then slow down). Schedule: attempts 0-4 (the first ~5
  // wakes, roughly 5 minutes at the 1-wake/minute cadence) retry every
  // wake; after that, retry only once every 5 wakes (roughly every 5
  // minutes) to conserve battery while a longer outage resolves.
  // dataFetchRetryCount persists across deep sleep (RTC_DATA_ATTR) so
  // this schedule is tracked correctly across many wake cycles, and
  // resets to 0 the moment a fetch actually succeeds (handled both here
  // and in attemptInitialDataFetch()'s caller in the freshBoot block
  // above).
  if (!haveData && !freshBoot) {
    const int FAST_RETRY_COUNT = 5;
    const int SLOW_RETRY_INTERVAL_WAKES = 5;
    bool shouldRetryNow;
    if (dataFetchRetryCount < FAST_RETRY_COUNT) {
      shouldRetryNow = true;
    } else {
      int wakesSinceFastPhase = dataFetchRetryCount - FAST_RETRY_COUNT;
      shouldRetryNow = (wakesSinceFastPhase % SLOW_RETRY_INTERVAL_WAKES) == 0;
    }

    if (shouldRetryNow) {
      Serial.printf("Retrying initial data fetch (attempt count=%d, %s phase)\n",
        dataFetchRetryCount, dataFetchRetryCount < FAST_RETRY_COUNT ? "fast" : "slow");
      bool retryOk = attemptInitialDataFetch();
      if (retryOk) {
        Serial.println("Retry succeeded -- data fetch recovered");
        dataFetchRetryCount = 0;
        logRetry = "attempted/succeeded";
        // Recompute nowUTC/now in case NTP just corrected the clock
        // significantly (fresh boot's own code doesn't need this since
        // it happens before now/nowUTC are first read, but this retry
        // runs AFTER they were already captured above).
        time(&now);
        nowUTC = now;
      } else {
        Serial.println("Retry failed -- will try again per backoff schedule");
        logRetry = "attempted/failed";
      }
    } else {
      logRetry = "skipped(backoff)";
    }
    dataFetchRetryCount++;
  }

  if (haveData && nowUTC > sunsetEpoch) {
    Serial.println("Past sunset — Jewish day rolled over, re-fetching");
    time_t oldSunset = sunsetEpoch;
    bool rolloverOk = false;
    bool wifiOk = wifiConnect();
    if (wifiOk) {
      time_t tomorrow = now + 86400;
      struct tm tt; localtime_r(&tomorrow,&tt);
      int gy=tt.tm_year+1900, gm=tt.tm_mon+1, gd=tt.tm_mday;
      bool okZ = fetchZmanim(gy,gm,gd);
      fetchHebrewDate(gy,gm,gd);
      fetchOmerDay(gy,gm,gd);
      fetchNextHoliday(gy,gm,gd,nowUTC); // always -- also refreshes parasha
      moonFrac = calcMoonFraction(gy,gm,gd);
      // Refresh the Torah Quotes CSV once per Jewish day, piggybacking
      // on this same WiFi connection -- no new trigger mechanism
      // needed since the sunset rollover already fires once per day.
      // Fetch failures leave the previous cached copy in place (see
      // fetchTorahQuotesCSV()'s own comments).
      fetchTorahQuotesCSV();
      wifiOff();
      if (okZ && sunsetEpoch > oldSunset) {
        rolloverOk = true;
        time(&lastSyncEpoch);
        time(&lastSuccessfulUpdateEpoch);
      } else {
        Serial.println("Rollover fetch did not produce a newer sunset — will retry next wake");
      }
    } else {
      Serial.println("WiFi failed during sunset rollover — will retry next wake");
    }
    if (!rolloverOk) {
      haveData = false;
    }
    logRollover = String(wifiOk ? "wifiOK" : "wifiFAIL") + "/" + (rolloverOk ? "rolloverOK" : "rolloverFAIL");
  }

  if (!freshBoot) {
    if (wakesSinceWeatherFetch >= 30) {
      bool wifiOk = wifiConnect();
      bool weatherOk = false;
      if (wifiOk) {
        weatherOk = fetchWeather();
        wifiOff();
      }
      logWeather = String(wifiOk ? "wifiOK" : "wifiFAIL") + "/" + (weatherOk ? "fetchOK" : "fetchFAIL");
      wakesSinceWeatherFetch = 0;
    } else {
      wakesSinceWeatherFetch++;
    }
  }

  String heroTime = haveData ? computeHalachicTime(nowUTC) : "—";
  Serial.printf("Halachic time: %s\n", heroTime.c_str());

  int battPct = readBatteryPct();
  bool charging = isCharging();
  Serial.printf("Charging: %s\n", charging ? "yes" : "no");
  bool syncFresh = haveData && lastSyncEpoch > 0 &&
                   (now - lastSyncEpoch) < SYNC_STALE_SECONDS;
  Serial.printf("Sync fresh: %s (%ld sec since last)\n",
    syncFresh ? "yes" : "no", (long)(now - lastSyncEpoch));

  bool doFullRefresh = freshBoot || screenChanged || forceRefreshOnly || (wakesSinceFullRefresh >= 60);
  if (doFullRefresh) wakesSinceFullRefresh = 0;
  else               wakesSinceFullRefresh++;

  if (doFullRefresh) {
    display.setFullWindow();
    display.firstPage();
    do { drawCurrentScreen(nowUTC, heroTime, battPct, syncFresh, charging); } while (display.nextPage());
  } else {
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.firstPage();
    do { drawCurrentScreen(nowUTC, heroTime, battPct, syncFresh, charging); } while (display.nextPage());
  }
  Serial.printf("Rendered screen %d (%s). Sleeping.\n", currentScreen, doFullRefresh ? "full" : "partial");

  {
    const char* wakeReason = freshBoot ? "freshBoot" : (wake == ESP_SLEEP_WAKEUP_EXT1 ? "button" : "timer");
    String line = String(nowUTC) + "\t" + wakeReason +
      "\trollover=" + logRollover +
      "\tweather=" + logWeather +
      "\tretry=" + logRetry +
      "\tsyncBefore=" + String((long)lastSyncBefore) +
      "\tsyncAfter=" + String((long)lastSyncEpoch) +
      "\tsyncFresh=" + (syncFresh ? "yes" : "no") +
      "\tbattPct=" + String(battPct) +
      "\tmoonFrac=" + String(moonFrac, 3);
    wakeLogAppend(line);
  }

  pinMode(MENU_BTN_PIN,INPUT_PULLUP);
  pinMode(BACK_BTN_PIN,INPUT_PULLUP);
  pinMode(UP_BTN_PIN,INPUT_PULLUP);
  pinMode(DOWN_BTN_PIN,INPUT_PULLUP);
  Serial.flush();
  time_t nowForSleep; time(&nowForSleep);
  struct tm stm; localtime_r(&nowForSleep, &stm);
  int secsToNextMin = 60 - stm.tm_sec;
  if (secsToNextMin <= 0) secsToNextMin = 60;
  esp_sleep_enable_timer_wakeup((uint64_t)secsToNextMin * 1000000ULL);
  esp_sleep_enable_ext1_wakeup(BTN_PIN_MASK, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_deep_sleep_start();
}

void loop(){}