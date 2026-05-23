#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_APDS9960.h>
#include <FirebaseESP32.h>
#include <math.h>
#include <time.h>

// =============================================================
// Totositter - BlackBoard Wisdom firmware
//
//   F1 Sound:    bargraph volume meter + bark trigger at 75% peak
//   F2 Movement: APDS-9960 proximity delta -> RGB blue
//   F3 Correlation: bark + movement within 2s -> RGB red (ALERT)
//   F4 Auto-feed: every 60s + button manual feed -> RGB green + OLED
// =============================================================

// ---- Pin map (BlackBoard Wisdom) ----
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t PIN_MIC     = 36;   // ADC1_CH0 (input only)
constexpr uint8_t PIN_LED_25  = 13;
constexpr uint8_t PIN_LED_50  = 4;
constexpr uint8_t PIN_LED_75  = 16;
constexpr uint8_t PIN_LED_100 = 17;
constexpr uint8_t PIN_RGB_R   = 19;
constexpr uint8_t PIN_RGB_G   = 23;
constexpr uint8_t PIN_RGB_B   = 18;
constexpr uint8_t PIN_BUTTON  = 27;
constexpr uint8_t PIN_BUZZER  = 25;   // passive piezo on servo connector

// ---- F1 - Mic ----
constexpr uint32_t MIC_SAMPLE_PERIOD_US = 125;   // ~8 kHz
constexpr uint32_t MIC_WINDOW_SAMPLES   = 400;   // ~50 ms @ 8 kHz
constexpr uint32_t MIC_CALIBRATION_MS   = 1500;
constexpr float    MIC_BARK_RATIO       = 0.75f;
constexpr uint32_t BARK_HOLD_MS         = 500;
constexpr uint32_t BARK_REFRACTORY_MS   = 1000;

// ---- F2 - Proximity ----
constexpr uint32_t PROX_POLL_MS         = 50;
constexpr uint8_t  PROX_DELTA_THRESHOLD = 15;
constexpr uint32_t MOVE_HOLD_MS         = 500;
constexpr uint32_t MOVE_REFRACTORY_MS   = 500;

// ---- F3 - Correlation ----
constexpr uint32_t CORRELATION_WINDOW_MS = 2000;
constexpr uint32_t CORRELATION_HOLD_MS   = 1500;

// ---- F5 - Snack (auto-treat on bark+movement) ----
constexpr uint32_t SNACK_COOLDOWN_MS = 5000;  // min 5s between snacks
constexpr uint32_t SNACK_FLASH_MS    = 500;
constexpr uint32_t SNACK_BANNER_MS   = 2000;

// ---- F6 - Buzzer (Brahms' Lullaby on bark) ----
// Note frequencies (only the ones the song uses)
constexpr int NOTE_G4 = 392;
constexpr int NOTE_A4 = 440;
constexpr int NOTE_B4 = 494;
constexpr int NOTE_C5 = 523;
constexpr int NOTE_D5 = 587;

// Tempo: whole note = 3000ms -> quarter = 750ms = 80 BPM (lullaby pace)
constexpr uint32_t MELODY_WHOLE_NOTE_MS = 3000;
constexpr uint16_t MELODY_NOTE_GAP_MS   = 30;

static const int BARK_MELODY_NOTES[] = {
  NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_B4, NOTE_C5, NOTE_C5, NOTE_B4, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4,
  NOTE_G4, NOTE_A4, NOTE_G4, NOTE_B4, NOTE_C5, NOTE_C5,
  NOTE_B4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_G4
};
// 4 = quarter, 8 = eighth, 2 = half (Arduino convention: ms = whole/value)
static const int BARK_MELODY_DURATIONS[] = {
  8, 8, 4, 8, 8, 4,
  8, 8, 4, 8, 8, 4,
  8, 8, 4, 8, 8, 4,
  8, 8, 4, 8, 8, 4,
  8, 8, 4, 2, 2
};
constexpr size_t BARK_MELODY_LEN =
  sizeof(BARK_MELODY_NOTES) / sizeof(BARK_MELODY_NOTES[0]);

// ---- F4 - Feeder ----
constexpr uint32_t FEED_INTERVAL_MS    = 60000;
constexpr uint32_t FEED_FLASH_MS       = 300;
constexpr uint32_t FEED_BANNER_MS      = 2000;
constexpr uint32_t BUTTON_DEBOUNCE_MS  = 50;

// ---- OLED ----
constexpr uint8_t  OLED_I2C_ADDR     = 0x3C;
constexpr uint16_t OLED_W            = 128;
constexpr uint16_t OLED_H            = 64;
constexpr uint32_t DISPLAY_UPDATE_MS = 100;

// =============================================================
// Network / Firebase credentials  --  REPLACE BEFORE FLASHING
// =============================================================
constexpr const char* WIFI_SSID     = "uaifai-brum";
constexpr const char* WIFI_PASSWORD = "bemvindoaocesar";

// Firebase Realtime Database (legacy database secret auth)
//   Database URL: console -> Realtime Database -> copy the https URL
//   Legacy token: console -> Project Settings -> Service Accounts -> Database Secrets
//   WARNING: this token grants full DB read/write. Do not commit publicly.
constexpr const char* FIREBASE_DATABASE_URL  = "https://totositter-9be96-default-rtdb.firebaseio.com/"; 
constexpr const char* FIREBASE_LEGACY_TOKEN  = "El2ALqJyrFjKLqOJUb50sJlo612yMNQxyHareEYO";

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint32_t FIREBASE_TASK_PERIOD_MS = 250;
constexpr uint32_t HEARTBEAT_PERIOD_MS     = 5000;
constexpr const char* FW_VERSION           = "0.1.0";

// NTP - sync ESP32 clock to wall time so every Firebase request can carry
// a real epoch timestamp instead of just boot-relative millis().
constexpr const char* NTP_SERVER_1    = "pool.ntp.org";
constexpr const char* NTP_SERVER_2    = "time.nist.gov";
constexpr long        NTP_GMT_OFFSET  = 0;     // store UTC; format on the client
constexpr int         NTP_DST_OFFSET  = 0;
constexpr uint32_t    NTP_SYNC_TIMEOUT_MS = 5000;
// time(2) earlier than this means NTP hasn't synced yet (Jan 1 2024).
constexpr time_t      EPOCH_MIN_VALID = 1704067200;

// =============================================================
// Shared state (cross-core; volatile is acceptable at this scope)
// =============================================================
struct AppState {
  // F1
  volatile uint8_t  mic_level    = 0;       // 0..4 bargraph steps
  volatile float    mic_rms      = 0.0f;
  volatile float    mic_peak     = 50.0f;
  volatile uint32_t last_bark_ms = 0;

  // F2
  volatile uint8_t  prox_value   = 0;
  volatile uint32_t last_move_ms = 0;

  // F3
  volatile uint32_t last_corr_ms   = 0;
  volatile bool     correlation_on = false;

  // F4
  volatile uint32_t feed_count     = 0;
  volatile uint32_t last_feed_ms   = 0;
  volatile bool     feed_flash_on  = false;
  volatile bool     feed_banner_on = false;
  char              last_feed_source[8] = "init";  // "auto" | "button"

  // F5 - Snack
  volatile uint32_t snack_count      = 0;
  volatile uint32_t last_snack_ms    = 0;
  volatile bool     snack_flash_on   = false;
  volatile bool     snack_banner_on  = false;

  // Recomputed each loop (visual hold flags)
  bool bark_active = false;
  bool move_active = false;
};

static AppState           g_state;
static Adafruit_SSD1306   oled(OLED_W, OLED_H, &Wire, -1);
static SparkFun_APDS9960  apds;
static FirebaseData       fbdo;
static FirebaseAuth       fb_auth;
static FirebaseConfig     fb_config;
static volatile bool      wifi_ok     = false;
static volatile bool      firebase_ok = false;

// =============================================================
// F1 - Mic sampling task (Core 1)
// =============================================================
static void mic_task(void* /*arg*/) {
  uint32_t cal_end = millis() + MIC_CALIBRATION_MS;
  uint64_t sum = 0;
  uint32_t count = 0;
  uint16_t maxv = 0, minv = 4095;
  while (millis() < cal_end) {
    uint16_t v = analogRead(PIN_MIC);
    sum += v; count++;
    if (v > maxv) maxv = v;
    if (v < minv) minv = v;
    delayMicroseconds(MIC_SAMPLE_PERIOD_US);
  }
  float baseline = count > 0 ? (float)sum / count : 2048.0f;
  float swing = (float)max((int)(maxv - baseline), (int)(baseline - minv));
  if (swing < 50.0f) swing = 50.0f;
  g_state.mic_peak = swing;

  for (;;) {
    double sq_sum = 0.0;
    for (uint32_t i = 0; i < MIC_WINDOW_SAMPLES; i++) {
      int32_t centered = (int32_t)analogRead(PIN_MIC) - (int32_t)baseline;
      sq_sum += (double)(centered * centered);
      delayMicroseconds(MIC_SAMPLE_PERIOD_US);
    }
    float rms = sqrtf((float)(sq_sum / MIC_WINDOW_SAMPLES));
    if (rms > g_state.mic_peak) g_state.mic_peak = rms;
    g_state.mic_rms = rms;

    float ratio = rms / g_state.mic_peak;
    if (ratio > 1.0f) ratio = 1.0f;

    uint8_t lvl = 0;
    if (ratio >= 0.25f) lvl = 1;
    if (ratio >= 0.50f) lvl = 2;
    if (ratio >= 0.75f) lvl = 3;
    if (ratio >= 0.95f) lvl = 4;
    g_state.mic_level = lvl;

    uint32_t now = millis();
    if (ratio >= MIC_BARK_RATIO &&
        (now - g_state.last_bark_ms) > BARK_REFRACTORY_MS) {
      g_state.last_bark_ms = now;
    }
  }
}

// =============================================================
// F2 - Proximity polling
// =============================================================
static void proximity_poll(uint32_t now_ms) {
  static uint32_t last_poll_ms = 0;
  static uint8_t  prev_prox    = 0;
  if (now_ms - last_poll_ms < PROX_POLL_MS) return;
  last_poll_ms = now_ms;

  uint8_t v = 0;
  if (!apds.readProximity(v)) return;
  g_state.prox_value = v;

  int delta = (int)v - (int)prev_prox;
  if (abs(delta) >= PROX_DELTA_THRESHOLD &&
      (now_ms - g_state.last_move_ms) > MOVE_REFRACTORY_MS) {
    g_state.last_move_ms = now_ms;
  }
  prev_prox = v;
}

// =============================================================
// F3 - Bark + Movement correlation
// =============================================================
static void correlation_update(uint32_t now_ms) {
  bool bark_recent = g_state.last_bark_ms != 0 &&
                     (now_ms - g_state.last_bark_ms) < CORRELATION_WINDOW_MS;
  bool move_recent = g_state.last_move_ms != 0 &&
                     (now_ms - g_state.last_move_ms) < CORRELATION_WINDOW_MS;

  if (bark_recent && move_recent && !g_state.correlation_on) {
    g_state.correlation_on = true;
    g_state.last_corr_ms   = now_ms;

    // F5 - trigger snack on correlation rising edge, respecting cooldown
    if (g_state.last_snack_ms == 0 ||
        (now_ms - g_state.last_snack_ms) > SNACK_COOLDOWN_MS) {
      g_state.snack_count++;
      g_state.last_snack_ms    = now_ms;
      g_state.snack_flash_on   = true;
      g_state.snack_banner_on  = true;
      Serial.printf("[%lu ms] Snack delivered - count=%lu reason=bark+movement\n",
                    (unsigned long)now_ms,
                    (unsigned long)g_state.snack_count);
    } else {
      uint32_t wait = SNACK_COOLDOWN_MS - (now_ms - g_state.last_snack_ms);
      Serial.printf("[%lu ms] Snack skipped - cooldown %lums remaining\n",
                    (unsigned long)now_ms, (unsigned long)wait);
    }
  }
  if (g_state.correlation_on &&
      (now_ms - g_state.last_corr_ms) > CORRELATION_HOLD_MS) {
    g_state.correlation_on = false;
  }

  if (g_state.snack_flash_on &&
      (now_ms - g_state.last_snack_ms) > SNACK_FLASH_MS) {
    g_state.snack_flash_on = false;
  }
  if (g_state.snack_banner_on &&
      (now_ms - g_state.last_snack_ms) > SNACK_BANNER_MS) {
    g_state.snack_banner_on = false;
  }
}

// =============================================================
// F4 - Feeder
// =============================================================
static void feeder_trigger(uint32_t now_ms, const char* source) {
  g_state.feed_count++;
  g_state.last_feed_ms   = now_ms;
  g_state.feed_flash_on  = true;
  g_state.feed_banner_on = true;
  strncpy(g_state.last_feed_source, source, sizeof(g_state.last_feed_source) - 1);
  g_state.last_feed_source[sizeof(g_state.last_feed_source) - 1] = '\0';
  Serial.printf("[%lu ms] Feeding pet - count=%lu source=%s\n",
                (unsigned long)now_ms,
                (unsigned long)g_state.feed_count,
                source);
}

static void feeder_update(uint32_t now_ms) {
  if (now_ms - g_state.last_feed_ms >= FEED_INTERVAL_MS) {
    feeder_trigger(now_ms, "auto");
  }

  static int      last_raw       = HIGH;
  static int      stable         = HIGH;
  static uint32_t last_change_ms = 0;
  int raw = digitalRead(PIN_BUTTON);
  if (raw != last_raw) { last_raw = raw; last_change_ms = now_ms; }
  if (now_ms - last_change_ms > BUTTON_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) feeder_trigger(now_ms, "button");
  }

  if (g_state.feed_flash_on &&
      (now_ms - g_state.last_feed_ms) > FEED_FLASH_MS) {
    g_state.feed_flash_on = false;
  }
  if (g_state.feed_banner_on &&
      (now_ms - g_state.last_feed_ms) > FEED_BANNER_MS) {
    g_state.feed_banner_on = false;
  }
}

// =============================================================
// F6 - Buzzer (plays melody on every bark detection)
// =============================================================
static void buzzer_play_melody() {
  for (size_t i = 0; i < BARK_MELODY_LEN; i++) {
    int duration_ms = MELODY_WHOLE_NOTE_MS / BARK_MELODY_DURATIONS[i];
    tone(PIN_BUZZER, BARK_MELODY_NOTES[i]);
    delay(duration_ms);
    noTone(PIN_BUZZER);
    delay(MELODY_NOTE_GAP_MS);
  }
}

static void buzzer_task(void* /*arg*/) {
  Serial.println("[BUZZ] task started");
  uint32_t last_played_bark = 0;
  for (;;) {
    uint32_t bark = g_state.last_bark_ms;
    if (bark != 0 && bark != last_played_bark) {
      Serial.printf("[BUZZ] playing arpeggio for bark @ %lu\n",
                    (unsigned long)bark);
      buzzer_play_melody();
      last_played_bark = bark;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =============================================================
// WiFi
// =============================================================
static void wifi_connect() {
  Serial.printf("WiFi connecting to %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_ok = true;
    Serial.printf("\nWiFi OK, IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed - continuing offline");
  }
}

// =============================================================
// NTP - wall-clock time for Firebase payloads
// =============================================================
static void ntp_sync() {
  if (!wifi_ok) {
    Serial.println("[NTP] skipped - no WiFi");
    return;
  }
  configTime(NTP_GMT_OFFSET, NTP_DST_OFFSET, NTP_SERVER_1, NTP_SERVER_2);
  uint32_t t0 = millis();
  time_t now = 0;
  while ((millis() - t0) < NTP_SYNC_TIMEOUT_MS) {
    time(&now);
    if (now >= EPOCH_MIN_VALID) break;
    delay(200);
  }
  if (now >= EPOCH_MIN_VALID) {
    Serial.printf("[NTP] synced epoch=%ld (%lums)\n",
                  (long)now, (unsigned long)(millis() - t0));
  } else {
    Serial.printf("[NTP] sync FAILED after %lums - ts will be 0 until sync\n",
                  (unsigned long)(millis() - t0));
  }
}

// Returns current epoch seconds (UTC), or 0 if NTP has not synced yet.
// Callers should treat 0 as "unknown" rather than a real timestamp.
static uint32_t now_epoch() {
  time_t now = 0;
  time(&now);
  return now >= EPOCH_MIN_VALID ? (uint32_t)now : 0;
}

// =============================================================
// Firebase
// =============================================================
static void firebase_init() {
  if (!wifi_ok) {
    Serial.println("[FB] init skipped - no WiFi");
    return;
  }

  // ---- Log config so we can see what's being sent ----
  size_t tok_len = strlen(FIREBASE_LEGACY_TOKEN);
  Serial.printf("[FB] config url=%s\n", FIREBASE_DATABASE_URL);
  if (tok_len >= 8) {
    Serial.printf("[FB] config token len=%u preview=%.4s...%.4s\n",
                  (unsigned)tok_len,
                  FIREBASE_LEGACY_TOKEN,
                  FIREBASE_LEGACY_TOKEN + tok_len - 4);
  } else {
    Serial.printf("[FB] config token len=%u (TOO SHORT - check it)\n",
                  (unsigned)tok_len);
  }
  Serial.printf("[FB] WiFi gateway=%s DNS=%s rssi=%d\n",
                WiFi.gatewayIP().toString().c_str(),
                WiFi.dnsIP().toString().c_str(),
                (int)WiFi.RSSI());

  fb_config.database_url               = FIREBASE_DATABASE_URL;
  fb_config.signer.tokens.legacy_token = FIREBASE_LEGACY_TOKEN;
  Firebase.begin(&fb_config, &fb_auth);
  Firebase.reconnectWiFi(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);

  // ---- Wait for ready (legacy token usually instant, but log progress) ----
  uint32_t t0 = millis();
  while (!Firebase.ready() && (millis() - t0) < 5000) {
    Serial.printf("[FB] waiting ready... %lums\n",
                  (unsigned long)(millis() - t0));
    delay(500);
  }
  if (!Firebase.ready()) {
    Serial.printf("[FB] init FAILED - Firebase.ready()=false after %lums\n",
                  (unsigned long)(millis() - t0));
    Serial.println("[FB] likely causes: wrong token, wrong URL/region, "
                   "DB rules deny writes, or SSL/network issue");
    firebase_ok = false;
    return;
  }
  firebase_ok = true;
  Serial.printf("[FB] Firebase.ready() OK in %lums\n",
                (unsigned long)(millis() - t0));

  // ---- Ping write to validate end-to-end connectivity ----
  FirebaseJson ping;
  ping.set("ts",        (int)now_epoch());
  ping.set("uptime_ms", (int)millis());
  ping.set("fw",        FW_VERSION);
  ping.set("ip",        WiFi.localIP().toString());
  ping.set("rssi",      (int)WiFi.RSSI());
  ping.set("message",   "Totositter online");
  Serial.println("[FB] ping -> /totositter/_ping");
  uint32_t pt0 = millis();
  if (Firebase.setJSON(fbdo, "/totositter/_ping", ping)) {
    Serial.printf("[FB] ping OK in %lums - verify in console at /totositter/_ping\n",
                  (unsigned long)(millis() - pt0));
  } else {
    Serial.printf("[FB] ping FAIL reason=\"%s\" httpCode=%d (%lums)\n",
                  fbdo.errorReason().c_str(), fbdo.httpCode(),
                  (unsigned long)(millis() - pt0));
    Serial.println("[FB] DIAGNOSE:");
    Serial.println("[FB]   httpCode 401/403  -> token invalid or rules deny");
    Serial.println("[FB]   httpCode 404      -> wrong database URL (check region)");
    Serial.println("[FB]   httpCode -1/-2    -> network/SSL failure");
    Serial.println("[FB]   reason  'token'   -> legacy token rejected by server");
  }
}

static String fb_events_path() { return String("/totositter/events"); }
static String fb_state_path()  { return String("/totositter/state");  }

static const char* current_state_label() {
  if (g_state.correlation_on) return "alert";
  if (g_state.bark_active && g_state.move_active) return "alert";
  if (g_state.bark_active) return "bark";
  if (g_state.move_active) return "move";
  return "idle";
}

static void firebase_log_event(const char* type, FirebaseJson* extra, uint32_t uptime_ms) {
  if (!firebase_ok || !Firebase.ready()) {
    Serial.printf("[FB] skip %s uptime=%lu (firebase_ok=%d ready=%d)\n",
                  type, (unsigned long)uptime_ms,
                  (int)firebase_ok, (int)Firebase.ready());
    return;
  }
  FirebaseJson json;
  json.set("type",      type);
  json.set("ts",        (int)now_epoch());
  json.set("uptime_ms", (int)uptime_ms);
  if (extra) json.set("data", *extra);
  String path = fb_events_path();
  uint32_t t0 = millis();
  if (Firebase.pushJSON(fbdo, path.c_str(), json)) {
    Serial.printf("[FB] OK push %s id=%s path=%s (%lums)\n",
                  type, fbdo.pushName().c_str(), path.c_str(),
                  (unsigned long)(millis() - t0));
  } else {
    Serial.printf("[FB] FAIL push %s reason=\"%s\" httpCode=%d (%lums)\n",
                  type, fbdo.errorReason().c_str(),
                  fbdo.httpCode(), (unsigned long)(millis() - t0));
  }
}

static void firebase_log_boot() {
  if (!firebase_ok || !Firebase.ready()) return;
  FirebaseJson extra;
  extra.set("fw",   FW_VERSION);
  extra.set("ip",   WiFi.localIP().toString());
  extra.set("rssi", (int)WiFi.RSSI());
  firebase_log_event("boot", &extra, millis());
}

static void firebase_update_state(uint32_t now_ms) {
  if (!firebase_ok || !Firebase.ready()) {
    Serial.printf("[FB] skip heartbeat (firebase_ok=%d ready=%d)\n",
                  (int)firebase_ok, (int)Firebase.ready());
    return;
  }
  FirebaseJson json;
  json.set("ts",             (int)now_epoch());
  json.set("uptime_ms",      (int)now_ms);
  json.set("state",          current_state_label());
  json.set("mic_rms",        (double)g_state.mic_rms);
  json.set("mic_peak",       (double)g_state.mic_peak);
  json.set("mic_level",      (int)g_state.mic_level);
  json.set("prox",           (int)g_state.prox_value);
  json.set("feed_count",     (int)g_state.feed_count);
  json.set("snack_count",    (int)g_state.snack_count);
  json.set("last_feed_ms",   (int)g_state.last_feed_ms);
  json.set("last_snack_ms",  (int)g_state.last_snack_ms);
  json.set("last_bark_ms",   (int)g_state.last_bark_ms);
  json.set("last_move_ms",   (int)g_state.last_move_ms);
  json.set("bark_active",    g_state.bark_active);
  json.set("move_active",    g_state.move_active);
  json.set("correlation_on", g_state.correlation_on);
  json.set("rssi",           (int)WiFi.RSSI());
  String path = fb_state_path();
  uint32_t t0 = millis();
  if (Firebase.setJSON(fbdo, path.c_str(), json)) {
    Serial.printf("[FB] OK heartbeat state=%s rssi=%d feeds=%lu (%lums)\n",
                  current_state_label(), (int)WiFi.RSSI(),
                  (unsigned long)g_state.feed_count,
                  (unsigned long)(millis() - t0));
  } else {
    Serial.printf("[FB] FAIL heartbeat reason=\"%s\" httpCode=%d (%lums)\n",
                  fbdo.errorReason().c_str(), fbdo.httpCode(),
                  (unsigned long)(millis() - t0));
  }
}

static void firebase_task(void* /*arg*/) {
  Serial.println("[FB] task started");
  uint32_t last_logged_bark  = 0;
  uint32_t last_logged_move  = 0;
  uint32_t last_logged_feed  = 0;
  uint32_t last_logged_corr  = 0;
  uint32_t last_logged_snack = 0;
  uint32_t last_heartbeat    = 0;
  const char* last_state    = "init";
  bool boot_logged          = false;
  bool prev_ready           = false;
  bool prev_wifi            = false;

  for (;;) {
    bool wifi_now  = (WiFi.status() == WL_CONNECTED);
    bool ready_now = firebase_ok && Firebase.ready();

    if (wifi_now != prev_wifi) {
      Serial.printf("[FB] WiFi status -> %s (rssi=%d)\n",
                    wifi_now ? "CONNECTED" : "DISCONNECTED",
                    wifi_now ? (int)WiFi.RSSI() : 0);
      prev_wifi = wifi_now;
    }
    if (ready_now != prev_ready) {
      Serial.printf("[FB] ready -> %d\n", (int)ready_now);
      prev_ready = ready_now;
    }

    if (ready_now) {
      uint32_t now = millis();

      if (!boot_logged) {
        firebase_log_boot();
        boot_logged = true;
      }

      uint32_t bark = g_state.last_bark_ms;
      if (bark != 0 && bark != last_logged_bark) {
        firebase_log_event("bark", nullptr, bark);
        last_logged_bark = bark;
      }

      uint32_t move = g_state.last_move_ms;
      if (move != 0 && move != last_logged_move) {
        FirebaseJson extra;
        extra.set("proximity", (int)g_state.prox_value);
        firebase_log_event("movement", &extra, move);
        last_logged_move = move;
      }

      uint32_t feed = g_state.last_feed_ms;
      if (feed != 0 && feed != last_logged_feed && g_state.feed_count > 0) {
        FirebaseJson extra;
        extra.set("source", g_state.last_feed_source);
        extra.set("count",  (int)g_state.feed_count);
        firebase_log_event("feed", &extra, feed);
        last_logged_feed = feed;
      }

      uint32_t corr = g_state.last_corr_ms;
      if (corr != 0 && corr != last_logged_corr) {
        firebase_log_event("alert", nullptr, corr);
        last_logged_corr = corr;
      }

      uint32_t snack = g_state.last_snack_ms;
      if (snack != 0 && snack != last_logged_snack) {
        FirebaseJson extra;
        extra.set("count",  (int)g_state.snack_count);
        extra.set("reason", "bark+movement");
        firebase_log_event("snack", &extra, snack);
        last_logged_snack = snack;
      }

      const char* cur = current_state_label();
      if (strcmp(cur, last_state) != 0) {
        FirebaseJson extra;
        extra.set("from", last_state);
        extra.set("to",   cur);
        firebase_log_event("state_change", &extra, now);
        last_state = cur;
      }

      if (now - last_heartbeat >= HEARTBEAT_PERIOD_MS) {
        firebase_update_state(now);
        last_heartbeat = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(FIREBASE_TASK_PERIOD_MS));
  }
}

// =============================================================
// LEDs
// =============================================================
static void leds_init() {
  pinMode(PIN_LED_25,  OUTPUT);
  pinMode(PIN_LED_50,  OUTPUT);
  pinMode(PIN_LED_75,  OUTPUT);
  pinMode(PIN_LED_100, OUTPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
}

static void bargraph_update() {
  uint8_t lvl = g_state.mic_level;
  digitalWrite(PIN_LED_25,  lvl >= 1 ? HIGH : LOW);
  digitalWrite(PIN_LED_50,  lvl >= 2 ? HIGH : LOW);
  digitalWrite(PIN_LED_75,  lvl >= 3 ? HIGH : LOW);
  digitalWrite(PIN_LED_100, lvl >= 4 ? HIGH : LOW);
}

static void rgb_update() {
  // Priority: snack (yellow=R+G) > correlation (red) > feed (green) > movement (blue)
  bool r = false, g = false, b = false;
  if (g_state.snack_flash_on)           { r = true; g = true; }
  else if (g_state.correlation_on)      { r = true; }
  else if (g_state.feed_flash_on)       { g = true; }
  else if (g_state.move_active)         { b = true; }
  digitalWrite(PIN_RGB_R, r ? HIGH : LOW);
  digitalWrite(PIN_RGB_G, g ? HIGH : LOW);
  digitalWrite(PIN_RGB_B, b ? HIGH : LOW);
}

// =============================================================
// OLED
// =============================================================
static bool display_init() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("OLED init failed");
    return false;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("Totositter");
  oled.println("Calibrating mic...");
  oled.display();
  return true;
}

static void display_update(uint32_t now_ms) {
  static uint32_t last_update_ms = 0;
  if (now_ms - last_update_ms < DISPLAY_UPDATE_MS) return;
  last_update_ms = now_ms;

  oled.clearDisplay();

  if (g_state.snack_banner_on) {
    // Full-screen takeover - snack delivered
    oled.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(28, 8);
    oled.print("SNACK!");
    oled.setTextSize(1);
    oled.setCursor(8, 32);
    oled.print("bark + movement");
    oled.setCursor(46, 50);
    oled.print("#");
    oled.print(g_state.snack_count);
  } else if (g_state.feed_banner_on) {
    // Full-screen takeover while feeding
    oled.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(16, 12);
    oled.print("Feeding");
    oled.setCursor(40, 32);
    oled.print("pet!");
    oled.setTextSize(1);
    oled.setCursor(46, 54);
    oled.print("#");
    oled.print(g_state.feed_count);
  } else {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("Bark:");
    oled.print(g_state.bark_active ? "YES" : "-- ");
    oled.print(" Move:");
    oled.println(g_state.move_active ? "YES" : "-- ");

    oled.print("F:");
    oled.print(g_state.feed_count);
    oled.print("  S:");
    oled.println(g_state.snack_count);

    uint32_t elapsed   = now_ms - g_state.last_feed_ms;
    uint32_t remaining = (FEED_INTERVAL_MS > elapsed) ? (FEED_INTERVAL_MS - elapsed) : 0;
    oled.print("Next feed: ");
    oled.print(remaining / 1000);
    oled.println("s");

    oled.print("State: ");
    if (g_state.correlation_on)                            oled.println("ALERT");
    else if (g_state.bark_active && !g_state.move_active)  oled.println("BARK");
    else if (!g_state.bark_active && g_state.move_active)  oled.println("MOVE");
    else                                                   oled.println("idle");
  }
  oled.display();
}

// =============================================================
// Setup / Loop
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Totositter booting...");

  leds_init();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!display_init())                     Serial.println("Display unavailable");
  if (!apds.init())                        Serial.println("APDS-9960 init failed");
  if (!apds.enableProximitySensor(false))  Serial.println("APDS-9960 prox enable failed");

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MIC, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);
  analogReadResolution(12);
  g_state.last_feed_ms = millis();

  wifi_connect();
  ntp_sync();
  firebase_init();

  xTaskCreatePinnedToCore(mic_task,      "mic",  4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(buzzer_task,   "buzz", 2048, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(firebase_task, "fb",   8192, nullptr, 1, nullptr, 0);
  Serial.println("Boot complete");
}

void loop() {
  uint32_t now = millis();

  proximity_poll(now);
  feeder_update(now);
  correlation_update(now);

  g_state.bark_active = g_state.last_bark_ms != 0 &&
                        (now - g_state.last_bark_ms) < BARK_HOLD_MS;
  g_state.move_active = g_state.last_move_ms != 0 &&
                        (now - g_state.last_move_ms) < MOVE_HOLD_MS;

  bargraph_update();
  rgb_update();
  display_update(now);

  delay(10);
}
