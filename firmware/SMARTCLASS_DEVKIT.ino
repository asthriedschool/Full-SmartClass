#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// ============================================================
// FILE INI SAJA untuk ESP32 DevKit. Jangan buka folder firmware lain.
// Arduino: File -> Open -> firmware/SMARTCLASS_DEVKIT/SMARTCLASS_DEVKIT.ino
// Board: ESP32 Dev Module. Serial 115200, Newline.
// Relay: COM+NO. Serial: UJI relay-2   atau   CAMIP 10.77.198.225
// ============================================================

// =====================================================
// 1. WIFI DAN BACKEND
// =====================================================

const char* WIFI_SSID = "kuks";
const char* WIFI_PASSWORD = "1234567890";

// Backend Spring Boot di laptop, HTTP port 8081. Bukan Nevacloud.
// Endpoint IoT: /api/iot/heartbeat, /api/iot/commands/claim, /ack, /pir-events
const char* API_BASE_URL =
  "http://10.240.129.206:8081";

#define FIRMWARE_VERSION "smartclass-devkit-2.2.1"
#define OTA_BOARD "devkit"
// Token update lewat internet (OTA). Jangan disebarkan.
#define OTA_TOKEN "GANTI_DENGAN_OTA_TOKEN_BARU_JIKA_OTA_DIGUNAKAN"

const char* CONTROLLER_CODE = "ESP32-CLASS-01";
const char* CONTROLLER_KEY =
  "AnscIKXFeWskm1KmXKEbEODrofIHPAXIfVVxaqTdi2g";

const long ROOM_ID = 2;

// ESP32-CAM dicari lewat mDNS dulu (IP hotspot bisa berubah), lalu cadangan CAMERA_URL.
const char* CAMERA_MDNS_NAME = "smartclass-cam";   // sama dengan CAM_MDNS_NAME di sketch kamera
const char* CAMERA_URL =
  "http://10.77.198.225/capture";
// Bisa diganti tanpa reflash: ketik di Serial  CAMIP 192.168.x.x
String cameraCaptureUrl = CAMERA_URL;

const char* CAMERA_TRIGGER_KEY =
  "SmartClass-Camera-2026-Manado-8xK92pLm";

// =====================================================
// 2. HARDWARE PIN CONFIGURATION
// =====================================================

#define PIR_PIN 27

// HC-SR501 / AM312 biasanya HIGH = ada gerakan.
// Jika Serial raw=1 saat diam dan raw=0 saat kamu gerak, ubah jadi false.
const bool PIR_ACTIVE_HIGH = true;

struct RelayConfig {
  const char* channel;
  uint8_t pin;
};

RelayConfig relays[] = {
  {"relay-1", 26},
  {"relay-2", 25},
  {"relay-3", 33},
  {"relay-4", 32},
  {"relay-5", 23}
};

const uint8_t RELAY_COUNT =
  sizeof(relays) / sizeof(relays[0]);

// Modul 5-relay umum: IN aktif LOW. COM+NO. HIGH = mati, LOW = nyala.
const bool RELAY_ACTIVE_LOW = false;

int relayOutputLevel(bool turnOn) {
  if (RELAY_ACTIVE_LOW) {
    return turnOn ? LOW : HIGH;
  }
  return turnOn ? HIGH : LOW;
}

void applyCommand(int pin, const String& command) {
  digitalWrite(pin, relayOutputLevel(command == "ON"));
}

// =====================================================
// 3. INTERVAL CONFIGURATION
// =====================================================

const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long COMMAND_POLL_INTERVAL = 1500;
const unsigned long WIFI_RETRY_INTERVAL = 15000;

const unsigned long PIR_WARMUP_INTERVAL = 20000;
const unsigned long PIR_DEBOUNCE_INTERVAL = 50;
const unsigned long PIR_DEBUG_INTERVAL = 1000;
const unsigned long PIR_REFRESH_INTERVAL = 15000;
const unsigned long CAMERA_RETRY_INTERVAL = 20000;
const unsigned long CAMERA_HTTP_TIMEOUT = 5000;

// =====================================================
// 4. GLOBAL STATE
// =====================================================

unsigned long bootAt = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastCommandPoll = 0;
unsigned long lastWifiAttempt = 0;

unsigned long pirChangedAt = 0;
unsigned long lastPirDebug = 0;
unsigned long lastWarmupLog = 0;
unsigned long lastPirEventAt = 0;
unsigned long lastCameraTriggerAt = 0;

int previousPirReading = LOW;
int stablePirState = LOW;

bool pirInitialized = false;
bool heartbeatSent = false;
bool wasWifiConnected = false;
bool wifiConnecting = false;

// =====================================================
// 5. WIFI MANAGEMENT
// =====================================================

void startWiFi() {
  Serial.println("[WiFi] Connecting...");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lastWifiAttempt = millis();
  wifiConnecting = true;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnecting = false;
    return;
  }

  if (wifiConnecting &&
      millis() - lastWifiAttempt < WIFI_RETRY_INTERVAL) {
    return;
  }

  if (lastWifiAttempt != 0 &&
      millis() - lastWifiAttempt < WIFI_RETRY_INTERVAL) {
    return;
  }

  if (wifiConnecting) {
    Serial.println("[WiFi] Retry...");
    WiFi.disconnect(true, true);
    delay(200);
  }

  startWiFi();
}

void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    return;
  }

  wifiConnecting = false;

  if (!wasWifiConnected) {
    wasWifiConnected = true;

    Serial.println();
    Serial.println("[WiFi] Connected!");

    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("[WiFi] Signal RSSI: ");
    Serial.println(WiFi.RSSI());

    MDNS.end();
    MDNS.begin("smartclass-relay");
  }
}

// =====================================================
// 6. BACKEND HTTP REQUEST
// =====================================================

WiFiClient apiClient;
HTTPClient apiHttp;

int controllerRequest(
  const char* method,
  const String& endpoint,
  const String& body,
  String& response
) {
  if (WiFi.status() != WL_CONNECTED) {
    return -1;
  }

  apiHttp.setReuse(true);
  apiHttp.setConnectTimeout(8000);
  apiHttp.setTimeout(10000);

  String fullUrl =
    String(API_BASE_URL) + endpoint;

  if (!apiHttp.begin(apiClient, fullUrl)) {
    Serial.println("[HTTP] Initialization failed");
    return -1;
  }

  apiHttp.addHeader("X-Controller-Code", CONTROLLER_CODE);
  apiHttp.addHeader("X-Controller-Key", CONTROLLER_KEY);
  apiHttp.addHeader("Content-Type", "application/json");

  int code = -1;

  if (strcmp(method, "POST") == 0) {
    code = apiHttp.POST(body);
  } else if (strcmp(method, "GET") == 0) {
    code = apiHttp.GET();
  }

  if (code == 204 || code == 304) {
    // Tidak ada isi. JANGAN panggil getString(): tanpa Content-Length ia
    // menunggu sampai koneksi ditutup server, inilah penyebab delay perintah.
    response = "";
  } else if (code > 0) {
    response = apiHttp.getString();
  } else {
    response = apiHttp.errorToString(code);
    apiClient.stop();
  }

  if (code != 204 && code != 200) {
    Serial.printf("[HTTP] %s %s -> %d\n", method, endpoint.c_str(), code);
  }

  apiHttp.end();
  return code;
}

// =====================================================
// 7. HEARTBEAT
// =====================================================

void sendHeartbeat() {
  String response;

  int code = controllerRequest(
    "POST",
    "/api/iot/heartbeat",
    "",
    response
  );

  if (code == 200) {
    Serial.println("[Heartbeat] Controller ONLINE");
    heartbeatSent = true;
  } else {
    Serial.printf("[Heartbeat] Failed: HTTP %d\n", code);
    if (response.length() > 0) {
      Serial.println(response);
    }
  }
}

// =====================================================
// 8. RELAY MAPPING
// =====================================================

int getRelayPin(const char* channel) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (strcmp(relays[i].channel, channel) == 0) {
      return relays[i].pin;
    }
  }
  return -1;
}

// =====================================================
// 9. RELAY CONTROL
// =====================================================

bool setRelay(const char* channel, const char* command) {
  int pin = getRelayPin(channel);

  if (pin < 0) {
    Serial.println("[Relay] Unknown relay channel");
    return false;
  }

  bool turnOn;

  if (strcmp(command, "ON") == 0) {
    turnOn = true;
  } else if (strcmp(command, "OFF") == 0) {
    turnOn = false;
  } else {
    Serial.println("[Relay] Invalid command");
    return false;
  }

  int output = relayOutputLevel(turnOn);
  digitalWrite(pin, output);

  Serial.printf(
    "[Relay] %s -> %s | GPIO %d | Output %s\n",
    channel,
    command,
    pin,
    output == HIGH ? "HIGH" : "LOW"
  );

  return true;
}

// =====================================================
// 10. COMMAND ACKNOWLEDGMENT
// =====================================================

bool acknowledgeCommand(
  long commandId,
  const String& claimToken,
  const char* status,
  const String& errorMessage
) {
  unsigned long acknowledgmentStartedAt = millis();
  JsonDocument doc;

  doc["claimToken"] = claimToken;
  doc["status"] = status;

  if (strcmp(status, "FAILED") == 0) {
    doc["errorMessage"] = errorMessage;
  }

  String body;
  serializeJson(doc, body);

  String endpoint =
    "/api/iot/commands/" +
    String(commandId) +
    "/ack";

  for (int attempt = 1; attempt <= 3; attempt++) {
    String response;
    unsigned long attemptStartedAt = millis();

    int code = controllerRequest(
      "POST",
      endpoint,
      body,
      response
    );

    if (code == 200) {
      Serial.printf(
        "[ACK] Command %ld: %s acknowledged t=%lu ms attempt=%d elapsed=%lu ms total=%lu ms\n",
        commandId,
        status,
        millis(),
        attempt,
        millis() - attemptStartedAt,
        millis() - acknowledgmentStartedAt
      );
      return true;
    }

    Serial.printf(
      "[ACK] Attempt %d failed: HTTP %d elapsed=%lu ms total=%lu ms\n",
      attempt,
      code,
      millis() - attemptStartedAt,
      millis() - acknowledgmentStartedAt
    );

    if (response.length() > 0) {
      Serial.println(response);
    }

    delay(1000);
  }

  Serial.println("[ACK] All acknowledgment attempts failed");
  return false;
}

// =====================================================
// 11. CLAIM COMMAND FROM BACKEND
// =====================================================

bool processCommand() {
  String response;
  unsigned long pollStartedAt = millis();

  int code = controllerRequest(
    "POST",
    "/api/iot/commands/claim",
    "",
    response
  );
  Serial.printf(
    "[Poll] /claim t=%lu ms HTTP=%d elapsed=%lu ms\n",
    pollStartedAt,
    code,
    millis() - pollStartedAt
  );

  if (code == 204) {
    return false;
  }

  if (code != 200) {
    Serial.printf("[Claim] HTTP %d\n", code);
    return false;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);

  if (error) {
    Serial.println("[Claim] Invalid JSON response");
    return false;
  }

  long commandId = doc["id_command"] | 0;
  unsigned long commandReceivedAt = millis();
  String channel = doc["relay_channel"] | "";
  String command = doc["command"] | "";
  String claimToken = doc["claim_token"] | "";
  String source = doc["source"] | "";

  if (commandId <= 0 || channel.isEmpty() || claimToken.isEmpty()) {
    Serial.println("[Claim] Incomplete response");
    return false;
  }

  Serial.println();
  Serial.printf(
    "[Claim] New command received t=%lu ms after poll start=%lu ms\n",
    commandReceivedAt,
    commandReceivedAt - pollStartedAt
  );
  Serial.printf("Command ID: %ld\n", commandId);
  Serial.printf("Relay: %s\n", channel.c_str());
  Serial.printf("Action: %s\n", command.c_str());
  if (source.length() > 0) {
    Serial.printf("Source: %s\n", source.c_str());
  }

  unsigned long relayStartedAt = millis();
  bool executed = setRelay(channel.c_str(), command.c_str());
  Serial.printf(
    "[Relay] Command %ld execution complete t=%lu ms elapsed=%lu ms success=%d\n",
    commandId,
    millis(),
    millis() - relayStartedAt,
    executed
  );

  if (executed) {
    acknowledgeCommand(commandId, claimToken, "SUCCESS", "");
  } else {
    acknowledgeCommand(
      commandId,
      claimToken,
      "FAILED",
      "Unknown relay channel or invalid command"
    );
  }
  return true;
}

// =====================================================
// 12. PIR EVENT TO BACKEND
// =====================================================

void sendPirEvent(bool motionDetected) {
  unsigned long eventStartedAt = millis();
  JsonDocument doc;

  doc["roomId"] = ROOM_ID;
  doc["motionDetected"] = motionDetected;

  String body;
  serializeJson(doc, body);

  String response;

  int code = controllerRequest(
    "POST",
    "/api/iot/pir-events",
    body,
    response
  );

  if (code == 200) {
    Serial.printf(
      "[PIR] Event sent: %s t=%lu ms elapsed=%lu ms\n",
      motionDetected ? "MOTION" : "NO MOTION",
      millis(),
      millis() - eventStartedAt
    );
  } else {
    Serial.printf(
      "[PIR] Event failed: HTTP %d elapsed=%lu ms\n",
      code,
      millis() - eventStartedAt
    );
    if (response.length() > 0) {
      Serial.println(response);
    }
  }
}

// =====================================================
// 13. TRIGGER ESP32-CAM
// =====================================================

void triggerCamera() {
  unsigned long triggerStartedAt = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Camera] WiFi disconnected");
    return;
  }

  IPAddress camIp = MDNS.queryHost(CAMERA_MDNS_NAME, 1500);
  if (camIp != IPAddress(0, 0, 0, 0)) {
    cameraCaptureUrl = "http://" + camIp.toString() + "/capture";
    Serial.printf("[Camera] mDNS OK %s\n", camIp.toString().c_str());
  } else {
    Serial.printf(
      "[Camera] mDNS skip/gagal, pakai %s\n",
      cameraCaptureUrl.c_str()
    );
  }
  String url = cameraCaptureUrl +
    "?key=" +
    CAMERA_TRIGGER_KEY +
    "&motion=1";

  Serial.println();
  Serial.println("[Camera] Sending capture trigger...");
  Serial.printf("[Camera] Target %s\n", cameraCaptureUrl.c_str());

  int code = -1;
  for (int attempt = 1; attempt <= 3; attempt++) {
    WiFiClient client;
    HTTPClient http;
    http.setConnectTimeout(3000);
    http.setTimeout(CAMERA_HTTP_TIMEOUT);
    if (!http.begin(client, url)) {
      Serial.println("[Camera] HTTP initialization failed");
      break;
    }
    code = http.GET();
    http.end();
    Serial.printf("[Camera] HTTP Response: %d (coba %d)\n", code, attempt);
    if (code == 200) {
      break;
    }
    delay(400);
  }
  Serial.printf(
    "[Camera] Trigger sequence complete t=%lu ms elapsed=%lu ms HTTP=%d\n",
    millis(),
    millis() - triggerStartedAt,
    code
  );
  if (code == 200) {
    Serial.println("[Camera] Trigger OK, camera queued capture");
  } else if (code == -11) {
    Serial.println("[Camera] Timeout (-11): kamera ketemu tapi sibuk upload. Upload firmware kamera terbaru.");
  } else {
    Serial.println("[Camera] Tidak terhubung ke ESP32-CAM.");
    Serial.println("[Camera] Buka Serial kamera, salin IP, lalu ketik: CAMIP x.x.x.x");
  }
  lastCameraTriggerAt = millis();
}

// =====================================================
// 14. PIR PROCESSING
// =====================================================

bool pirIsMotion(int reading) {
  return PIR_ACTIVE_HIGH ? reading == HIGH : reading == LOW;
}

void processPIR() {
  int reading = digitalRead(PIR_PIN);
  unsigned long now = millis();

  if (now - lastPirDebug >= PIR_DEBUG_INTERVAL) {
    lastPirDebug = now;
    Serial.printf(
      "[PIR] raw GPIO%d=%d motion=%s warmup=%s\n",
      PIR_PIN,
      reading,
      pirIsMotion(reading) ? "YES" : "NO",
      now - bootAt < PIR_WARMUP_INTERVAL ? "YES" : "NO"
    );
  }

  if (now - bootAt < PIR_WARMUP_INTERVAL) {
    if (now - lastWarmupLog >= 5000) {
      lastWarmupLog = now;
      unsigned long left =
          (PIR_WARMUP_INTERVAL - (now - bootAt)) / 1000;
      Serial.printf(
        "[PIR] Warmup %lus lagi, jangan tes dulu\n",
        left
      );
    }
    return;
  }

  if (reading != previousPirReading) {
    previousPirReading = reading;
    pirChangedAt = now;
  }

  if (now - pirChangedAt < PIR_DEBOUNCE_INTERVAL) {
    return;
  }

  if (!pirInitialized) {
    stablePirState = reading;
    pirInitialized = true;

    Serial.printf(
      "[PIR] Siap. GPIO=%d. Gerakkan tangan sekarang.\n",
      reading
    );

    sendPirEvent(pirIsMotion(stablePirState));
    lastPirEventAt = now;
    return;
  }

  bool motionNow = pirIsMotion(reading);
  bool motionWas = pirIsMotion(stablePirState);
  bool changed = reading != stablePirState;
  bool refreshMotion =
      motionNow &&
      (now - lastPirEventAt >= PIR_REFRESH_INTERVAL);
  bool retryCamera =
      motionNow &&
      (now - lastCameraTriggerAt >= CAMERA_RETRY_INTERVAL);

  if (!changed && !refreshMotion && !retryCamera) {
    return;
  }

  if (changed) {
    stablePirState = reading;
  }

  if (motionNow) {
    if (changed) {
      Serial.printf("\n[PIR] MOTION DETECTED t=%lu ms\n", now);
    }
    if (changed || refreshMotion) {
      sendPirEvent(true);
      lastPirEventAt = now;
    }
    if (changed || retryCamera) {
      triggerCamera();
    }
  } else if (changed) {
    Serial.println("[PIR] NO MOTION");
    sendPirEvent(false);
    lastPirEventAt = now;
  }
}

// =====================================================
// 14b. UJI RELAY DARI SERIAL MONITOR (115200, Newline)
//   UJI relay-2   -> relay-2 ON 3 detik lalu OFF
//   UJI SEMUA     -> relay-1..5 bergantian
// =====================================================

void testRelay(const char* channel) {
  int pin = getRelayPin(channel);
  if (pin < 0) {
    Serial.printf("[Uji] Channel %s tidak ada\n", channel);
    return;
  }
  Serial.printf("[Uji] %s (GPIO %d) ON 3 detik... dengar bunyi klik & lihat LED modul\n", channel, pin);
  setRelay(channel, "ON");
  Serial.printf("[Uji] Baca balik GPIO %d = %d\n", pin, digitalRead(pin));
  delay(3000);
  setRelay(channel, "OFF");
}

void processSerial() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("CAMIP") || line.startsWith("camip")) {
    String ip = line.substring(5);
    ip.trim();
    if (ip.length() < 7) {
      Serial.println("[Camera] Contoh: CAMIP 192.168.43.55");
      return;
    }
    cameraCaptureUrl = "http://" + ip + "/capture";
    Serial.printf("[Camera] URL cadangan = %s\n", cameraCaptureUrl.c_str());
    return;
  }
  if (!line.startsWith("UJI") && !line.startsWith("uji")) {
    return;
  }
  String arg = line.substring(3);
  arg.trim();
  if (arg.equalsIgnoreCase("SEMUA")) {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      testRelay(relays[i].channel);
      delay(1000);
    }
  } else {
    testRelay(arg.c_str());
  }
}

// =====================================================
// 15. SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  bootAt = millis();

  Serial.println();
  Serial.println("================================");
  Serial.println(" SMARTCLASS ESP32 DEVKIT " FIRMWARE_VERSION);
  Serial.println(" PIR + 5 RELAYS + ESP32-CAM");
  Serial.printf(" Backend: %s\n", API_BASE_URL);
  Serial.printf(
    " Relay: %s (LOW=ON)\n",
    RELAY_ACTIVE_LOW ? "ACTIVE_LOW" : "ACTIVE_HIGH"
  );
  Serial.println("================================");

  pinMode(PIR_PIN, INPUT);
  Serial.printf(
    "[PIR] GPIO %d | active %s\n",
    PIR_PIN,
    PIR_ACTIVE_HIGH ? "HIGH" : "LOW"
  );

  int offLevel = relayOutputLevel(false);

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    digitalWrite(relays[i].pin, offLevel);

    Serial.printf(
      "[Relay] %s | GPIO %d | Initialized OFF\n",
      relays[i].channel,
      relays[i].pin
    );
  }

  Serial.setTimeout(50);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  startWiFi();

  previousPirReading = digitalRead(PIR_PIN);

  Serial.println();
  Serial.println("[System] Setup completed");
  Serial.println("[System] Waiting for WiFi and PIR warmup...");
}

// =====================================================
// 16. MAIN LOOP
// =====================================================

void loop() {
  maintainWiFi();

  if (WiFi.status() != WL_CONNECTED) {
    wasWifiConnected = false;
    heartbeatSent = false;
    delay(50);
    return;
  }

  unsigned long now = millis();

  if (!heartbeatSent ||
      now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  if (now - lastCommandPoll >= COMMAND_POLL_INTERVAL) {
    lastCommandPoll = now;
    if (processCommand()) {
      // Masih mungkin ada perintah lain di antrean: ambil lagi segera.
      lastCommandPoll = 0;
    }
  }

  processPIR();
  processSerial();
  delay(10);
}
