
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>   // esp_wifi_set_ps() — disables power-save on ESP32-S2
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define DHTTYPE DHT11   // DHT 11

//Pin numbers are based on GPIO pins
#define DHTPin 21     // Digital pin connected to the DHT sensor
#define WATER_TANK_LIGHT 20
#define WATER_LINE_LIGHT 8
#define FAN 9

//Status LEDs
#define PWR_LED 10
#define WIFI_LED 2
#define MQTT_CONN_LED 3
#define MQTT_SEND_LED 4

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPin, DHTTYPE);

// ---------- CONFIG ----------
const char* ssid     = "yelwap";
const char* password = "9a8b7c6d5e";

// MQTT server connection variables
const char* mqttserver   = "192.168.1.127";
const char* mqttUsername = "yelwap";
const char* mqttPassword = "Spotman^93";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32-pumpheater";

//Set MQTT topic parameter
char pubTopic[] = "arduino/enviro";

// Time sync (NTP) — needed to stamp MQTT messages with real wall-clock time.
// Times are reported in US Central and switch between CST/CDT automatically.
// TZ_INFO is a POSIX TZ string; change it to report a different zone, e.g.
// "EST5EDT,M3.2.0,M11.1.0" (Eastern) or "UTC0" (UTC, no DST).
const char*   NTP_SERVER = "pool.ntp.org";
const char*   TZ_INFO    = "CST6CDT,M3.2.0,M11.1.0";  // US Central w/ auto DST

// Timing
const unsigned long SENSOR_READ_INTERVAL_MS = 5000;    // how often to sample DHT + run control logic
const unsigned long PUBLISH_INTERVAL_MS     = 300000;  // MQTT publish cadence (5 min, unchanged)
const unsigned long MQTT_SEND_LED_MS        = 250;     // send-LED blink duration (non-blocking)

// Diagnostics
const unsigned long WIFI_DIAG_INTERVAL_MS = 30000;     // periodic link-health heartbeat

// WiFi reconnection tuning
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;   // max time to wait for association
const unsigned long WIFI_BACKOFF_BASE_MS    = 2000;    // initial backoff before retry
const unsigned long WIFI_BACKOFF_MAX_MS     = 60000;   // cap on exponential backoff
const uint8_t       WIFI_MAX_RETRIES        = 10;      // resets backoff after this many failures

// MQTT reconnection tuning
const unsigned long MQTT_BACKOFF_BASE_MS    = 2000;    // initial backoff before retry
const unsigned long MQTT_BACKOFF_MAX_MS     = 60000;   // cap on exponential backoff
const uint8_t       MQTT_MAX_RETRIES        = 10;      // resets backoff after this many failures
// ---------- END CONFIG ----------

//Configure JSON read buffer. Bumped to 512 B to fit the embedded WiFi diag
//object; PubSubClient's buffer is enlarged to match in setup().
char buffer[512];
StaticJsonDocument<512> output;

//Create instance for WiFi client and initialize variables
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long fan_timer = 0;
float f;
boolean light_toggle = 0;
boolean r1_status = 0;
boolean r2_status = 0;
boolean fan_status = 0;

// Non-blocking loop timers
static unsigned long lastSensorRead  = 0;
static unsigned long mqttSendLedOnAt = 0;
static bool          mqttSendLedOn   = false;

// Flag set by the WiFi event callback; read and cleared in loop().
// Keeps all Serial/network calls out of the driver task context.
volatile bool wifiGotIP = false;

// --- WiFi diagnostics (all written in event context, read/cleared in loop) ---
// A disconnect event only sets a flag + stashes its reason code; loop() prints.
volatile bool     wifiJustDisconnected = false;
volatile uint8_t  wifiLastDisconnectReason = 0;
// Running counters / timers for link-health reporting.
static uint32_t      wifiDisconnectCount = 0;   // total drops since boot
static unsigned long wifiConnectedSince  = 0;   // millis() of last GOT_IP (0 = down)
static unsigned long wifiLastDiagReport  = 0;   // last heartbeat print

// MQTT reconnection state
static uint8_t       mqttRetryCount  = 0;
static unsigned long mqttBackoffMs   = MQTT_BACKOFF_BASE_MS;
static unsigned long mqttLastAttempt = 0;

// WiFi reconnection state
static uint8_t       wifiRetryCount   = 0;
static unsigned long wifiBackoffMs    = WIFI_BACKOFF_BASE_MS;
static unsigned long wifiLastCheck    = 0;
static unsigned long wifiConnectStart = 0;
static bool          wifiConnecting   = false;

//$$$$$$$$$$$$$$$$$$$$$$$$ END VARIABLE INITALIZATION $$$$$$$$$$$$$$$

//&&&&&&&&&&&&&&&&&&. WIFI / MQTT CONNECTION STRATEGY &&&&&&&&&&&&&&&&&&&&

// Decode the ESP32 WiFi disconnect reason code into a readable string.
// These map to wifi_err_reason_t in esp_wifi_types.h; the common culprits for
// a link that connects fine then drops are BEACON_TIMEOUT (200), AUTH_EXPIRE (2),
// ASSOC_EXPIRE (4) and the 4-way-handshake timeouts (15/16).
const char* wifiReasonStr(uint8_t reason) {
  switch (reason) {
    case 1:   return "UNSPECIFIED";
    case 2:   return "AUTH_EXPIRE";        // AP aged out our authentication
    case 3:   return "AUTH_LEAVE";
    case 4:   return "ASSOC_EXPIRE";       // AP aged out our association
    case 5:   return "ASSOC_TOOMANY";      // AP is full / rejecting clients
    case 6:   return "NOT_AUTHED";
    case 7:   return "NOT_ASSOCED";
    case 8:   return "ASSOC_LEAVE";        // we/AP deliberately disassociated
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
    case 16:  return "GROUP_KEY_UPDATE_TIMEOUT";
    case 23:  return "802_1X_AUTH_FAILED";
    case 24:  return "CIPHER_SUITE_REJECTED";
    case 200: return "BEACON_TIMEOUT";     // lost the AP's beacons (range/interference)
    case 201: return "NO_AP_FOUND";        // AP not visible during (re)connect
    case 202: return "AUTH_FAIL";          // wrong password / auth rejected
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    default:  return "OTHER";
  }
}

// WiFi event callback — runs in the WiFi driver task context.
// IMPORTANT: do NOT call Serial.print(), WiFi, or any blocking function here.
// Set flags only; loop() reads and acts on them safely.
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiGotIP = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      // ensureWiFi() detects loss via WiFi.status(); here we only stash the
      // driver-supplied reason code so loop() can log WHY the link dropped.
      wifiLastDisconnectReason = info.wifi_sta_disconnected.reason;
      wifiJustDisconnected     = true;
      break;
    default:
      break;
  }
}

// Blocking scan of visible 2.4 GHz APs — a diagnostic for NO_AP_FOUND (201).
// If our target SSID does NOT appear here but shows up on a phone, the usual
// cause is the AP sitting on channel 12/13 (the ESP32's default country code
// only scans 1-11), a 5 GHz-only broadcast, or a hidden SSID.
void scanForAP() {
  Serial.println("[Scan] Scanning for 2.4 GHz networks...");
  int n = WiFi.scanNetworks();  // blocking; ~2-3 s
  if (n <= 0) {
    Serial.println("[Scan] No networks found at all - radio/antenna or "
                   "country-code issue, or all APs on channels 12-13.");
    WiFi.scanDelete();
    return;
  }

  bool foundTarget = false;
  Serial.print("[Scan] ");
  Serial.print(n);
  Serial.println(" network(s):");
  for (int i = 0; i < n; i++) {
    bool isTarget = (WiFi.SSID(i) == ssid);
    if (isTarget) foundTarget = true;
    Serial.print("  ");
    Serial.print(isTarget ? "*> " : "   ");   // '*>' marks our target SSID
    Serial.print("ch ");
    Serial.print(WiFi.channel(i));
    Serial.print("\tRSSI ");
    Serial.print(WiFi.RSSI(i));
    Serial.print("\tenc ");
    Serial.print(WiFi.encryptionType(i));     // 0=open,3=WPA2_PSK,4=WPA_WPA2,6=WPA3 etc.
    Serial.print("\t");
    Serial.println(WiFi.SSID(i));
  }

  if (foundTarget) {
    Serial.print("[Scan] Target SSID '");
    Serial.print(ssid);
    Serial.println("' IS visible.");
  } else {
    Serial.print("[Scan] Target SSID '");
    Serial.print(ssid);
    Serial.println("' NOT visible to this radio. Check: AP channel (avoid "
                   "12/13), 2.4 GHz enabled, SSID not hidden, WPA2 available.");
  }
  WiFi.scanDelete();
}

// Non-blocking WiFi manager. Call from loop() on every iteration.
// Returns true when WiFi is (or just became) connected.
bool ensureWiFi() {
  const unsigned long now = millis();

  // Already connected — reset backoff state and return immediately.
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnecting) {
      // Just finished connecting
      wifiConnecting     = false;
      wifiRetryCount     = 0;
      wifiBackoffMs      = WIFI_BACKOFF_BASE_MS;
      wifiConnectedSince = now;
      // (Re)start NTP now that we have network; SNTP syncs in the background.
      // configTzTime applies the POSIX TZ rules so local time tracks DST.
      configTzTime(TZ_INFO, NTP_SERVER);
      digitalWrite(WIFI_LED, HIGH);
      Serial.print("[WiFi] Connected, IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(", RSSI ");
      Serial.print(WiFi.RSSI());
      Serial.print(" dBm, ch ");
      Serial.print(WiFi.channel());
      Serial.print(", BSSID ");
      Serial.println(WiFi.BSSIDstr());
    }
    return true;
  }

  // Mark LED off whenever we know we're disconnected.
  digitalWrite(WIFI_LED, LOW);

  // If a connection attempt is in progress, check for timeout.
  if (wifiConnecting) {
    if (now - wifiConnectStart >= WIFI_CONNECT_TIMEOUT_MS) {
      // Timed out — abort and schedule a backoff retry.
      WiFi.disconnect(true, true);  // true,true = also erase NVS credentials
      delay(100);
      WiFi.mode(WIFI_OFF);          // fully reset radio before next attempt
      delay(200);
      wifiConnecting = false;
      wifiRetryCount++;

      Serial.print("[WiFi] Timed out (attempt ");
      Serial.print(wifiRetryCount);
      Serial.print("). Backing off ");
      Serial.print(wifiBackoffMs / 1000);
      Serial.println("s.");

      // Exponential backoff, capped at WIFI_BACKOFF_MAX_MS.
      wifiBackoffMs = min(wifiBackoffMs * 2, WIFI_BACKOFF_MAX_MS);

      // After too many consecutive failures reset counters so the device
      // keeps retrying indefinitely rather than giving up permanently.
      if (wifiRetryCount >= WIFI_MAX_RETRIES) {
        Serial.println("[WiFi] Max retries reached; resetting backoff.");
        wifiRetryCount = 0;
        wifiBackoffMs  = WIFI_BACKOFF_BASE_MS;
      }

      wifiLastCheck = now;  // start of backoff window
    }
    // Still waiting — do nothing this tick.
    return false;
  }

  // Not currently connecting — check if the backoff window has elapsed.
  if (now - wifiLastCheck < wifiBackoffMs) {
    return false;
  }

  // Kick off a new connection attempt (non-blocking).
  Serial.print("[WiFi] Connecting to ");
  Serial.print(ssid);
  Serial.println(" ...");

  WiFi.mode(WIFI_STA);
  delay(100);                     // give S2 driver a moment after mode switch
  esp_wifi_set_ps(WIFI_PS_NONE);  // disable power-save — critical on ESP32-S2
  WiFi.begin(ssid, password);
  wifiConnecting   = true;
  wifiConnectStart = now;
  return false;
}

// Non-blocking MQTT manager. Call from loop() on every iteration.
// Returns true when MQTT is connected.
bool ensureMQTT() {
  if (client.connected()) {
    mqttRetryCount = 0;
    mqttBackoffMs  = MQTT_BACKOFF_BASE_MS;
    digitalWrite(MQTT_CONN_LED, HIGH);
    return true;
  }

  digitalWrite(MQTT_CONN_LED, LOW);

  const unsigned long now = millis();
  if (now - mqttLastAttempt < mqttBackoffMs) return false;

  mqttLastAttempt = now;

  Serial.print("[MQTT] Connecting to broker...");
  if (client.connect(MQTT_CLIENT_ID, mqttUsername, mqttPassword)) {
    digitalWrite(MQTT_CONN_LED, HIGH);
    mqttRetryCount = 0;
    mqttBackoffMs  = MQTT_BACKOFF_BASE_MS;
    Serial.println(" connected.");
    return true;
  }

  // Failed — back off
  mqttRetryCount++;
  Serial.print(" failed, rc=");
  Serial.print(client.state());
  Serial.print(". Retry ");
  Serial.print(mqttRetryCount);
  Serial.print(" in ");
  Serial.print(mqttBackoffMs / 1000);
  Serial.println("s.");

  mqttBackoffMs = min(mqttBackoffMs * 2, MQTT_BACKOFF_MAX_MS);

  if (mqttRetryCount >= MQTT_MAX_RETRIES) {
    Serial.println("[MQTT] Max retries reached; resetting backoff.");
    mqttRetryCount = 0;
    mqttBackoffMs  = MQTT_BACKOFF_BASE_MS;
  }

  return false;
}

//############################## BEGIN MAIN LOOPS ###################
void setup()
{
    pinMode(PWR_LED, OUTPUT);
    digitalWrite(PWR_LED, HIGH);
    pinMode(WIFI_LED, OUTPUT);
    pinMode(MQTT_CONN_LED, OUTPUT);
    pinMode(MQTT_SEND_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);
    digitalWrite(MQTT_CONN_LED, LOW);
    digitalWrite(MQTT_SEND_LED, LOW);

    // ESP32-S2 uses native USB CDC for Serial. setTxTimeoutMs(0) prevents
    // Serial.print() from blocking/locking when no Serial Monitor is open, and
    // guards against CDC lockup when WiFi driver tasks and Serial writes occur
    // concurrently at startup. The timed while-loop waits up to 3 s for a
    // monitor to attach so early boot messages are not lost when one IS present.
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    unsigned long _t = millis();
    while (!Serial && (millis() - _t) < 3000) { delay(10); }

    Serial.println("[Boot] Pump heater controller starting...");

    pinMode(DHTPin, INPUT);
    pinMode(WATER_TANK_LIGHT, OUTPUT);
    digitalWrite(WATER_TANK_LIGHT, LOW);
    pinMode(WATER_LINE_LIGHT, OUTPUT);
    digitalWrite(WATER_LINE_LIGHT, LOW);
    pinMode(FAN, OUTPUT);
    digitalWrite(FAN, LOW);
    dht.begin();

    // --- WiFi ---
    // Register the event handler before touching the radio.
    WiFi.onEvent(onWiFiEvent);

    // Fully reset the radio to clear any stale driver state from a warm boot.
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    // Prime the radio for the non-blocking connection that completes in loop().
    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_ps(WIFI_PS_NONE);  // disable power-save — critical on ESP32-S2

    // One-time diagnostic scan: shows exactly which APs (and channels) this
    // radio can see. Compare the list against what your phone sees.
    scanForAP();

    // Begin the initial connection attempt (non-blocking; completes in loop()).
    WiFi.begin(ssid, password);
    wifiConnecting   = true;
    wifiConnectStart = millis();
    Serial.print("[WiFi] Connecting to ");
    Serial.print(ssid);
    Serial.println(" ...");

    // --- MQTT ---
    client.setServer(mqttserver, MQTT_PORT);
    // PubSubClient's default buffer is 256 B; our payload (readings + diag
    // object) can exceed that, so publish() would silently return false and
    // send nothing. Enlarge the buffer to fit the full packet.
    client.setBufferSize(512);
}

void loop()
{
  unsigned long now = millis();

  // -------- Service the connection strategy on EVERY iteration --------
  // Doing this first (before any timed/early-return control code) guarantees
  // WiFi/MQTT are maintained continuously, which is the whole point of the
  // non-blocking managers.

  // Non-blocking turn-off for the MQTT "send" LED.
  if (mqttSendLedOn && (now - mqttSendLedOnAt >= MQTT_SEND_LED_MS)) {
    digitalWrite(MQTT_SEND_LED, LOW);
    mqttSendLedOn = false;
  }

  // Service the wifiGotIP flag here in loop() context — safe to print/log.
  if (wifiGotIP) {
    wifiGotIP = false;
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Service a disconnect event: log the driver's reason code and how long the
  // link had been up. This is the key clue for diagnosing WHY WiFi drops.
  if (wifiJustDisconnected) {
    wifiJustDisconnected = false;
    uint8_t reason = wifiLastDisconnectReason;
    wifiDisconnectCount++;
    Serial.print("[WiFi] DISCONNECTED - reason ");
    Serial.print(reason);
    Serial.print(" (");
    Serial.print(wifiReasonStr(reason));
    Serial.print("). Drop #");
    Serial.print(wifiDisconnectCount);
    if (wifiConnectedSince != 0) {
      Serial.print(", was up ");
      Serial.print((now - wifiConnectedSince) / 1000);
      Serial.print("s");
    }
    Serial.println(".");
    wifiConnectedSince = 0;  // link is down until the next GOT_IP
  }

  // Periodic link-health heartbeat — trends in RSSI and the drop count over
  // time show whether drops correlate with a weakening signal.
  if (now - wifiLastDiagReport >= WIFI_DIAG_INTERVAL_MS) {
    wifiLastDiagReport = now;
    Serial.print("[Diag] up ");
    Serial.print(now / 1000);
    Serial.print("s, WiFi ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("UP RSSI ");
      Serial.print(WiFi.RSSI());
      Serial.print("dBm");
      if (wifiConnectedSince != 0) {
        Serial.print(" (");
        Serial.print((now - wifiConnectedSince) / 1000);
        Serial.print("s)");
      }
    } else {
      Serial.print("DOWN");
    }
    Serial.print(", drops ");
    Serial.print(wifiDisconnectCount);
    Serial.print(", MQTT ");
    Serial.print(client.connected() ? "UP" : "DOWN");
    Serial.print(" (state ");
    Serial.print(client.state());
    Serial.print("), heap ");
    Serial.print(ESP.getFreeHeap());
    Serial.println("B");
  }

  // Non-blocking WiFi + MQTT maintenance.
  bool wifiOk = ensureWiFi();
  bool mqttOk = wifiOk ? ensureMQTT() : false;
  client.loop();

  // -------- Timed sensor read + control logic (non-blocking gate) --------
  if (now - lastSensorRead < SENSOR_READ_INTERVAL_MS) return;
  lastSensorRead = now;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor).
  // The DHT library caches a reading for ~2 s, so h/t/f below come from one
  // consistent sample without any blocking delays between them.
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  // Check if any reads failed and exit early (to try again next interval).
  if (isnan(h) || isnan(f) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Temperature: "));
  Serial.print(f);
  Serial.print(" degF");
  Serial.println();
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(" %");
  Serial.println();
  Serial.print(F("Heat Index: "));
  Serial.print(hif);
  Serial.println(" degF");
  Serial.println(("__________"));

  long now_fan = millis();
  //Turn Heat Lamp 1 On
  if ((f <= 40) and (r1_status == 0))
  {
    digitalWrite(WATER_TANK_LIGHT, HIGH);
    r1_status = 1;
    Serial.print(F("Light1 status(TURN ON TEST): "));
    Serial.println(r1_status);
  }

  //Turn Heat Lamp 2 On
  if ((f <= 36) and (r2_status == 0))
  {
    digitalWrite(WATER_LINE_LIGHT, HIGH);
    r2_status = 1;
    Serial.println(F("Light2 status(TURN ON TEST): "));
    Serial.println(r2_status);
    fan_timer = now_fan;
    Serial.println(fan_timer);
    light_toggle = 1;
  }

  //Turn Heat Lamp 2 Off
  if ((f >= 36) and (r2_status == 1))
  {
    digitalWrite(WATER_LINE_LIGHT, LOW);
    r2_status = 0;
    Serial.print(F("Light2 status(TURN OFF TEST): "));
    Serial.println(r2_status);
    fan_timer = now_fan;
    Serial.println(fan_timer);
    light_toggle = 1;
  }

  //Turn Heat Lamp 1 Off
  if ((f > 40) and (r1_status == 1))
  {
    digitalWrite(WATER_TANK_LIGHT, LOW);
    r1_status = 0;
    Serial.print(F("Light1 status(TURN OFF TEST): "));
    Serial.println(r1_status);
  }

  if (light_toggle == 1)
  {
    if (now_fan - fan_timer > 30000)
    {
      if (fan_status == true)
      {
        Serial.print(F("Fan status(TURN OFF TEST): "));
        fan_status = false;
        Serial.println(fan_status);
        digitalWrite(FAN, LOW);
        light_toggle = 0;
      }
      else
      {
        Serial.print(F("Fan status(TURN ON TEST): "));
        fan_status = true;
        Serial.println(fan_status);
        digitalWrite(FAN, HIGH);
        light_toggle = 0;
      }
    }
  }

  // Wall-clock timestamp. time() returns UTC seconds since 1970 (epoch is
  // timezone-independent); a value before ~2021 means NTP has not synced yet
  // (no network since boot), so we report 0/"" rather than a bogus 1970 date.
  // ts_iso is rendered in local Central time with its UTC offset (e.g. -0500).
  time_t nowEpoch = time(nullptr);
  bool   timeValid = (nowEpoch > 1609459200);  // 2021-01-01
  char   isoTime[30] = "";
  if (timeValid) {
    struct tm tmLocal;
    localtime_r(&nowEpoch, &tmLocal);
    strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%S%z", &tmLocal);
  }

  //Store DHT data into JSON read buffer
  output["ts"]            = timeValid ? (uint32_t)nowEpoch : 0;  // unix epoch seconds, 0 if unsynced
  output["ts_iso"]        = isoTime;                             // ISO-8601 local time, "" if unsynced
  output["temperature"]   = f;
  output["humidity"]      = h;
  output["heat_index"]    = hif;
  output["relay1_status"] = r1_status;
  output["relay2_status"] = r2_status;
  output["fan_status"]    = fan_status;

  // --- WiFi diagnostics embedded in the payload ---
  // These let you watch link health remotely (broker / dashboard) instead of
  // needing a serial cable. Correlate wifi_drops climbing with a falling rssi
  // to confirm a signal problem; wifi_last_reason tells you WHY it last dropped.
  JsonObject diag = output.createNestedObject("diag");
  diag["uptime_s"]             = now / 1000;
  diag["free_heap"]            = ESP.getFreeHeap();
  diag["rssi"]                 = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  diag["wifi_up_s"]            = (wifiConnectedSince != 0) ? (now - wifiConnectedSince) / 1000 : 0;
  diag["wifi_drops"]           = wifiDisconnectCount;
  diag["wifi_last_reason"]     = wifiLastDisconnectReason;
  diag["wifi_last_reason_str"] = wifiReasonStr(wifiLastDisconnectReason);

  size_t n = serializeJson(output, buffer, sizeof(buffer));

  // Publish on the same 5-minute cadence as before, but only when MQTT is up.
  if (mqttOk && (now - lastMsg > PUBLISH_INTERVAL_MS))
  {
    lastMsg = now;
    // Light the send LED (turned off non-blockingly at the top of loop()).
    digitalWrite(MQTT_SEND_LED, HIGH);
    mqttSendLedOn   = true;
    mqttSendLedOnAt = now;

    // publish() returns false without sending if the packet exceeds the client
    // buffer (see setBufferSize above) or the socket write fails — never let
    // that fail silently.
    if (!client.publish(pubTopic, buffer, n)) {
      Serial.print("[MQTT] publish FAILED (payload ");
      Serial.print(n);
      Serial.print(" B, buffer ");
      Serial.print(client.getBufferSize());
      Serial.print(" B, state ");
      Serial.print(client.state());
      Serial.println(").");
    }
  }

  Serial.println(("______________"));
  Serial.println();
}
//############################## END MAIN LOOPS ###################
