#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include "esp_camera.h"

#define FIRMWARE_VERSION "smartclass-cam-2.3.1"
#define OTA_BOARD "cam"
#define OTA_TOKEN "GANTI_DENGAN_OTA_TOKEN_BARU_JIKA_OTA_DIGUNAKAN"

// ============================================================
// FILE INI SAJA untuk ESP32-CAM. Jangan buka folder firmware lain.
// Arduino: File -> Open -> firmware/SMARTCLASS_CAM/SMARTCLASS_CAM.ino
// Board: AI Thinker ESP32-CAM. Serial 115200.
// Upload INI DULU, baru SMARTCLASS_DEVKIT.
// ============================================================

const char* WIFI_SSID = "kuks";
const char* WIFI_PASS = "1234567890";
// Backend Spring Boot di laptop (HTTP :8081). Ganti IP jika DHCP berubah.
// POST raw JPEG, query idRoom + motionDetected wajib agar YOLO jalan.
const char* UPLOAD_URL =
  // "http://172.30.169.206:8081/api/detection/upload";
  "http://10.240.129.206:8081/api/detection/upload";
const char* UPLOAD_KEY = "";

const uint32_t ROOM_ID = 2;
const bool ROTATE_180 = true;

// Harus sama dengan CAMERA_TRIGGER_KEY di sketch DevKit.
const char* CAMERA_TRIGGER_KEY =
  "SmartClass-Camera-2026-Manado-8xK92pLm";
// Nama mDNS: DevKit memanggil http://smartclass-cam.local/capture
const char* CAM_MDNS_NAME = "smartclass-cam";

const char* CONTROLLER_CODE = "ESP32-CLASS-01";
const char* CONTROLLER_KEY = "AnscIKXFeWskm1KmXKEbEODrofIHPAXIfVVxaqTdi2g";

// Jarak minimum antar-foto (ms) supaya backend/YOLO tidak kebanjiran.
const unsigned long MIN_CAPTURE_GAP_MS = 3000;
// Cadangan jika DevKit gagal memanggil /capture (HTTP -1 / hotspot isolasi).
// Capture periodik ditandai bukan sebagai trigger PIR, sehingga tidak dapat
// memenuhi syarat ON AUTO tanpa event PIR dan hasil YOLO dari capture terpicu.
const unsigned long PERIODIC_CAPTURE_MS = 8000;

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

WebServer server(80);

bool cameraReady = false;
bool cameraHasPsram = false;
portMUX_TYPE cameraMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool cameraBusy = false;
bool captureRequested = false;
bool requestedMotion = false;
unsigned long captureQueuedAt = 0;
unsigned long lastCaptureAt = 0;
unsigned long lastWifiAttempt = 0;
bool wasConnected = false;
int lastUploadCode = 0;
String lastUploadAt = "-";
uint32_t uploadOk = 0;
uint32_t uploadFail = 0;
uint32_t triggers = 0;

void startNetworkServices() {
  MDNS.end();
  if (MDNS.begin(CAM_MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
  }
  Serial.printf("[WiFi] Terhubung. IP kamera: %s  (http://%s.local)\n",
                WiFi.localIP().toString().c_str(), CAM_MDNS_NAME);
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wasConnected) {
      wasConnected = true;
      startNetworkServices();
    }
    return;
  }
  wasConnected = false;
  if (millis() - lastWifiAttempt < 15000 && lastWifiAttempt != 0) {
    return;
  }
  lastWifiAttempt = millis();
  Serial.println("[WiFi] Menyambung...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

bool initCamera() {
  cameraHasPsram = psramFound();
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // QVGA keeps JPEG frames within memory limits, including boards without PSRAM.
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 18;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  if (cameraHasPsram) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
  }
  esp_err_t error = esp_camera_init(&config);
  if (error != ESP_OK) {
    Serial.printf("[Camera] Init error: 0x%x\n", error);
    return false;
  }
  Serial.printf(
      "[Camera] Config: QVGA, JPEG quality=%u, buffer=1, PSRAM=%s\n",
      config.jpeg_quality,
      cameraHasPsram ? "yes" : "no");
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr && ROTATE_180) {
    sensor->set_vflip(sensor, 1);
    sensor->set_hmirror(sensor, 1);
  }
  return true;
}

// Ambil frame BARU: buang satu frame yang mungkin sudah lama di buffer.
camera_fb_t* grabFreshFrame() {
  camera_fb_t* old = esp_camera_fb_get();
  if (old != nullptr) {
    esp_camera_fb_return(old);
  }
  return esp_camera_fb_get();
}

void printYoloResult(
    const String& body,
    int httpCode,
    unsigned long httpRoundTripMs) {
  Serial.println("================================");
  Serial.println("SMARTCLASS YOLO RESULT");
  Serial.println("================================");
  Serial.println("Camera Capture : SUCCESS");

  JsonDocument response;
  DeserializationError parseError = deserializeJson(response, body);
  if (parseError) {
    Serial.printf(
        "Backend Response : HTTP %d, JSON INVALID (%s)\n",
        httpCode,
        parseError.c_str());
    Serial.println("Human Detected : N/A");
    Serial.println("Person Count : N/A");
    Serial.println("Confidence : N/A");
    Serial.println("Inference Time : N/A");
    Serial.printf("HTTP Round Trip : %lu ms\n", httpRoundTripMs);
    Serial.println("Model : N/A");
    Serial.println("================================");
    return;
  }

  Serial.printf("Backend Response : HTTP %d\n", httpCode);

  JsonVariantConst humanValue = response["humanDetected"];
  if (humanValue.is<bool>()) {
    const bool humanDetected = humanValue.as<bool>();
    Serial.printf("Human Detected : %s\n", humanDetected ? "YES" : "NO");

    JsonVariantConst countValue = response["personCount"];
    if (countValue.is<int>()) {
      Serial.printf("Person Count : %d\n", countValue.as<int>());
    } else {
      Serial.println("Person Count : N/A");
    }

    JsonVariantConst confidenceValue = response["confidence"];
    if (humanDetected && !confidenceValue.isNull()
        && (confidenceValue.is<double>() || confidenceValue.is<int>())) {
      Serial.printf(
          "Confidence : %.2f%%\n",
          confidenceValue.as<double>() * 100.0);
    } else {
      Serial.println("Confidence : N/A");
    }
  } else {
    Serial.println("Human Detected : N/A");
    Serial.println("Person Count : N/A");
    Serial.println("Confidence : N/A");
  }

  JsonVariantConst inferenceValue = response["inferenceTimeMs"];
  if (!inferenceValue.isNull()
      && (inferenceValue.is<double>() || inferenceValue.is<int>())) {
    Serial.printf(
        "Inference Time : %.2f ms\n",
        inferenceValue.as<double>());
  } else {
    Serial.println("Inference Time : N/A");
  }

  JsonVariantConst backendTimeValue = response["totalProcessingTimeMs"];
  if (!backendTimeValue.isNull()
      && (backendTimeValue.is<unsigned long>()
          || backendTimeValue.is<int>())) {
    const unsigned long backendTimeMs = backendTimeValue.as<unsigned long>();
    if (httpRoundTripMs >= backendTimeMs) {
      Serial.printf(
          "Upload Time (estimated) : %.2f ms\n",
          static_cast<double>(httpRoundTripMs - backendTimeMs));
    } else {
      Serial.println("Upload Time (estimated) : N/A");
    }
  } else {
    Serial.printf("HTTP Round Trip : %lu ms\n", httpRoundTripMs);
  }

  JsonVariantConst modelValue = response["modelName"];
  if (modelValue.is<const char*>()) {
    Serial.printf("Model : %s\n", modelValue.as<const char*>());
  } else {
    Serial.println("Model : N/A");
  }
  Serial.println("================================");
}

int uploadFrame(camera_fb_t* frame, bool motion) {
  if (WiFi.status() != WL_CONNECTED || frame == nullptr) {
    return -1;
  }
  String url = String(UPLOAD_URL) + "?idRoom=" + ROOM_ID
      + "&motionDetected=" + (motion ? "true" : "false");
  if (strlen(UPLOAD_KEY) > 0) {
    url += String("&key=") + UPLOAD_KEY;
  }
  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(20000);
  http.setReuse(false);
  if (!http.begin(client, url)) {
    return -1;
  }
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("X-Controller-Code", CONTROLLER_CODE);
  http.addHeader("X-Controller-Key", CONTROLLER_KEY);
  unsigned long uploadStartedAt = millis();
  Serial.printf(
      "[Upload] POST mulai t=%lu ms, motion=%d, bytes=%u\n",
      uploadStartedAt,
      motion,
      frame->len);
  int code = http.POST(frame->buf, frame->len);
  unsigned long httpRoundTripMs = millis() - uploadStartedAt;
  String body;
  if (code > 0 && code != 204) {
    body = http.getString();
    httpRoundTripMs = millis() - uploadStartedAt;
  }
  Serial.printf(
      "[Upload] Response selesai t=%lu ms, elapsed=%lu ms, HTTP %d\n",
      millis(),
      millis() - uploadStartedAt,
      code);
  if (code >= 200 && code < 300) {
    Serial.printf("[Upload] Backend HTTP %d\n", code);
    if (body.isEmpty()) {
      Serial.println("[YOLO] Backend tidak mengirim detail hasil deteksi.");
    } else {
      printYoloResult(body, code, httpRoundTripMs);
    }
  } else {
    Serial.printf("[Upload] Backend HTTP %d\n", code);
    if (body.isEmpty() && code < 0) {
      Serial.println("[Upload] Koneksi atau respons backend gagal.");
    } else if (!body.isEmpty()) {
      Serial.printf("[Upload] Backend error: %s\n", body.c_str());
    }
  }
  http.end();
  return code;
}

bool lockCamera(uint32_t timeoutMs) {
  unsigned long start = millis();
  for (;;) {
    portENTER_CRITICAL(&cameraMux);
    const bool got = !cameraBusy;
    if (got) {
      cameraBusy = true;
    }
    portEXIT_CRITICAL(&cameraMux);
    if (got) {
      return true;
    }
    if (timeoutMs == 0 || millis() - start >= timeoutMs) {
      return false;
    }
    delay(5);
  }
}

void unlockCamera() {
  portENTER_CRITICAL(&cameraMux);
  cameraBusy = false;
  portEXIT_CRITICAL(&cameraMux);
}

void queueCapture(bool motion) {
  portENTER_CRITICAL(&captureMux);
  if (!captureRequested) {
    captureQueuedAt = millis();
  }
  captureRequested = true;
  requestedMotion = requestedMotion || motion;
  portEXIT_CRITICAL(&captureMux);
}

bool takeCaptureRequest(bool& motion, unsigned long& queuedAt) {
  bool taken = false;
  portENTER_CRITICAL(&captureMux);
  if (captureRequested) {
    motion = requestedMotion;
    queuedAt = captureQueuedAt;
    captureRequested = false;
    requestedMotion = false;
    captureQueuedAt = 0;
    taken = true;
  }
  portEXIT_CRITICAL(&captureMux);
  return taken;
}

void sendJpeg(camera_fb_t* frame) {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(frame->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  size_t written = 0;
  while (written < frame->len) {
    const size_t n = client.write(frame->buf + written, frame->len - written);
    if (n == 0) {
      break;
    }
    written += n;
  }
}

void handleSnapshot() {
  if (server.arg("key") != CAMERA_TRIGGER_KEY) {
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"key salah\"}");
    return;
  }
  if (!cameraReady) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"camera_not_ready\"}");
    return;
  }
  // Jangan antri lama di handleClient: /capture harus tetap bisa dijawab.
  if (!lockCamera(0)) {
    server.send(409, "application/json", "{\"ok\":false,\"busy\":true}");
    return;
  }
  camera_fb_t* frame = grabFreshFrame();
  if (frame == nullptr) {
    unlockCamera();
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"capture_failed\"}");
    return;
  }
  sendJpeg(frame);
  esp_camera_fb_return(frame);
  unlockCamera();
}

void doCapture(bool motion, unsigned long queuedAt) {
  if (!cameraReady) {
    Serial.println("[Camera] Kamera belum siap");
    return;
  }
  if (!lockCamera(2000)) {
    Serial.println("[Camera] Sensor sibuk (live snapshot), foto YOLO ditunda");
    queueCapture(motion);
    return;
  }
  unsigned long captureStartedAt = millis();
  if (queuedAt != 0) {
    Serial.printf(
        "[Camera] Trigger-to-capture-start=%lu ms, motion=%d\n",
        captureStartedAt - queuedAt,
        motion);
  } else {
    Serial.printf(
        "[Camera] Periodic capture start t=%lu ms\n",
        captureStartedAt);
  }
  lastCaptureAt = millis();
  camera_fb_t* frame = grabFreshFrame();
  if (frame == nullptr) {
    unlockCamera();
    Serial.printf(
        "[Camera] Gagal mengambil gambar (PSRAM=%s, free heap=%u, free PSRAM=%u)\n",
        cameraHasPsram ? "yes" : "no",
        ESP.getFreeHeap(),
        ESP.getFreePsram());
    uploadFail++;
    return;
  }
  Serial.printf(
      "[Camera] Capture selesai elapsed=%lu ms, bytes=%u, motion=%d\n",
      millis() - captureStartedAt,
      frame->len,
      motion);
  int code = uploadFrame(frame, motion);
  esp_camera_fb_return(frame);
  unlockCamera();
  lastUploadCode = code;
  lastUploadAt = String(millis() / 1000) + " s sejak nyala";
  if (code >= 200 && code < 300) {
    uploadOk++;
  } else {
    uploadFail++;
  }
  Serial.printf("[Upload] HTTP %d\n", code);
}

void handleCapture() {
  if (server.arg("key") != CAMERA_TRIGGER_KEY) {
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"key salah\"}");
    return;
  }
  triggers++;
  bool motion = server.arg("motion") != "0";
  unsigned long triggerAt = millis();
  queueCapture(motion);
  // Jawab segera. Foto + upload di loop(); task HTTP tetap hidup saat POST ke VPS.
  Serial.printf(
      "[Camera] /capture trigger t=%lu ms queued, motion=%d\n",
      triggerAt,
      motion);
  server.send(200, "application/json", "{\"ok\":true,\"queued\":true}");
}

void handleStatus() {
  String json = String("{\"camera\":") + (cameraReady ? "true" : "false")
      + ",\"ip\":\"" + WiFi.localIP().toString() + "\""
      + ",\"rssi\":" + WiFi.RSSI()
      + ",\"triggers\":" + triggers
      + ",\"upload_ok\":" + uploadOk
      + ",\"upload_fail\":" + uploadFail
      + ",\"last_upload_http\":" + lastUploadCode
      + ",\"last_upload\":\"" + lastUploadAt + "\""
      + ",\"room\":" + ROOM_ID
      + ",\"snapshot\":\"/snapshot\"}";
  server.send(200, "application/json", json);
}

void cameraWebTask(void* /*unused*/) {
  for (;;) {
    server.handleClient();
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("SMARTCLASS ESP32-CAM " FIRMWARE_VERSION " (dipicu PIR)");
  cameraReady = initCamera();
  Serial.printf("[Camera] init %s\n", cameraReady ? "OK" : "GAGAL (cek kabel pita kamera / daya 5V 2A)");

  WiFi.persistent(false);
  maintainWifi();

  server.on("/capture", handleCapture);
  server.on("/snapshot", handleSnapshot);
  server.on("/status", handleStatus);
  server.on("/", handleStatus);
  server.begin();
  xTaskCreatePinnedToCore(cameraWebTask, "camweb", 8192, NULL, 3, NULL, 0);
  Serial.println("[Camera] /capture + /snapshot di core terpisah (live preview tidak lewat /capture)");
}

void loop() {
  maintainWifi();

  unsigned long now = millis();
  bool hasQueuedCapture;
  portENTER_CRITICAL(&captureMux);
  hasQueuedCapture = captureRequested;
  portEXIT_CRITICAL(&captureMux);
  if (hasQueuedCapture && now - lastCaptureAt >= MIN_CAPTURE_GAP_MS) {
    bool motion = false;
    unsigned long queuedAt = 0;
    if (takeCaptureRequest(motion, queuedAt)) {
      doCapture(motion, queuedAt);
    }
  } else if (PERIODIC_CAPTURE_MS > 0 &&
      now - lastCaptureAt >= PERIODIC_CAPTURE_MS) {
    doCapture(false, 0);
  }
  delay(2);
}
