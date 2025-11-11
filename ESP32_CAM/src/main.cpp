// XIAO ESP32S3 Sense — Camera streamer + GPS raw endpoint + button actions (FreeRTOS)

// ====== USER CONFIG ======
#define WIFI_SSID      "UB_Devices"
#define WIFI_PASS      "goubbulls"

// Optional outbound endpoints (device acts as HTTP client on button events)
// Leave empty "" to disable network sends
#define REPORT_URL     "http://192.168.1.100:8000/report" // backend ip
#define SOS_URL        "http://192.168.1.100:8000/sos" // backend ip
#define SOS_CANCEL_URL "http://192.168.1.100:8000/sos_cancel" // backend ip

// GPS UART pins and baud
#define GPS_RX_PIN   5   // connect GPS TX -> this RX
#define GPS_TX_PIN   6   // optional: connect GPS RX <- this TX
#define GPS_BAUD     9600

// Button + Buzzer pins
#define BUTTON_GPIO  1     // active-low button to GND, enable internal PULLUP
#define BUZZER_GPIO  2     // active-high buzzer

// ====== CAMERA PINS (XIAO ESP32S3 Sense w/ OV2640) ======
#include "esp_camera.h"
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
// ==========================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <HTTPClient.h>
#include <esp_timer.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

// ====== FreeRTOS ======
TaskHandle_t hTaskGPS, hTaskButtons, hTaskHeartbeat;
SemaphoreHandle_t gpsMutex;

// ====== GPS raw buffer ======
static const size_t GPS_BUF_SZ = 2048;
static volatile size_t gps_wr = 0;
static char gps_buf[GPS_BUF_SZ];

// ====== Status ======
static String macStr;
static IPAddress ipAddr;

// ====== Utils ======
static inline void buzzer_beep(uint16_t ms) {
  digitalWrite(BUZZER_GPIO, HIGH);
  vTaskDelay(pdMS_TO_TICKS(ms));
  digitalWrite(BUZZER_GPIO, LOW);
}

static String ipToStr(IPAddress ip) {
  char s[32];
  snprintf(s, sizeof(s), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(s);
}

static void http_send_simple(const char* url, const char* payload) {
  if (!url || !url[0]) return;
  HTTPClient http;
  http.setTimeout(2000);
  if (http.begin(url)) {
    http.addHeader("Content-Type", "text/plain");
    http.POST((uint8_t*)payload, strlen(payload));
    http.end();
  }
}

// ====== CAMERA HTTP (multipart MJPEG) ======
static httpd_handle_t httpd_handle = nullptr;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
  const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_ok = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      if (!jpeg_ok) {
        esp_camera_fb_return(fb);
        continue;
      }
    } else {
      _jpg_buf = fb->buf;
      _jpg_buf_len = fb->len;
    }

    if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned)_jpg_buf_len);
      if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK ||
          httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len) != ESP_OK) {
        res = ESP_FAIL;
      }
    }

    if (fb->format != PIXFORMAT_JPEG) free(_jpg_buf);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    // pace
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return res;
}

static esp_err_t status_handler(httpd_req_t *req) {
  String s = "{";
  s += "\"mac\":\"" + macStr + "\",";
  s += "\"ip\":\"" + ipToStr(ipAddr) + "\",";
  s += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  s += "\"uptime_ms\":" + String(millis());
  s += "}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, s.c_str());
  return ESP_OK;
}

static esp_err_t gps_latest_handler(httpd_req_t *req) {
  // return the tail of gps buffer as text/plain
  httpd_resp_set_type(req, "text/plain");
  xSemaphoreTake(gpsMutex, portMAX_DELAY);
  // Send from gps_wr rolling back up to 1KB
  size_t start = (gps_wr + GPS_BUF_SZ - 1024) % GPS_BUF_SZ;
  for (size_t i = 0; i < 1024; ++i) {
    char c = gps_buf[(start + i) % GPS_BUF_SZ];
    httpd_resp_send_chunk(req, &c, 1);
  }
  xSemaphoreGive(gpsMutex);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static void start_httpd() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.stack_size = 8192;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_handle = server;
    httpd_uri_t uri_stream = {
      .uri      = "/stream",
      .method   = HTTP_GET,
      .handler  = stream_handler,
      .user_ctx = NULL
    };
    httpd_uri_t uri_status = {
      .uri      = "/status",
      .method   = HTTP_GET,
      .handler  = status_handler,
      .user_ctx = NULL
    };
    httpd_uri_t uri_gps = {
      .uri      = "/gps",
      .method   = HTTP_GET,
      .handler  = gps_latest_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stream);
    httpd_register_uri_handler(server, &uri_status);
    httpd_register_uri_handler(server, &uri_gps);
  }
}

// ====== CAMERA INIT ======
static void camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 10;            // lower=better
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    // continue, but /stream will fail
  }
}

// ====== WiFi ======
static void wifi_init() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 15000) {
    vTaskDelay(pdMS_TO_TICKS(250));
    Serial.print('.');
  }
  Serial.println();
  macStr = WiFi.macAddress();
  ipAddr = WiFi.localIP();
  Serial.printf("MAC: %s\n", macStr.c_str());
  Serial.printf("IP : %s\n", ipToStr(ipAddr).c_str());
  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
}

// ====== GPS TASK ======
HardwareSerial GPSSerial(1);

static void gps_task(void *arg) {
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  for (;;) {
    while (GPSSerial.available()) {
      char c = (char)GPSSerial.read();
      xSemaphoreTake(gpsMutex, portMAX_DELAY);
      gps_buf[gps_wr] = c;
      gps_wr = (gps_wr + 1) % GPS_BUF_SZ;
      xSemaphoreGive(gpsMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ====== BUTTON TASK (single/double/triple click) ======
static uint8_t read_button() { return (uint8_t)digitalRead(BUTTON_GPIO); } // HIGH=idle (pullup), LOW=pressed

static void send_status_report() {
  String payload = "type=report&mac=" + macStr + "&ip=" + ipToStr(ipAddr) + "&rssi=" + String(WiFi.RSSI());
  http_send_simple(REPORT_URL, payload.c_str());
}

static void send_sos() {
  String payload = "type=sos&mac=" + macStr + "&ip=" + ipToStr(ipAddr);
  http_send_simple(SOS_URL, payload.c_str());
}

static void send_sos_cancel() {
  String payload = "type=sos_cancel&mac=" + macStr + "&ip=" + ipToStr(ipAddr);
  http_send_simple(SOS_CANCEL_URL, payload.c_str());
}

static void buttons_task(void *arg) {
  const uint32_t DEBOUNCE_MS = 25;
  const uint32_t MULTI_GAP_MS = 400; // max gap between clicks
  bool prev = true;
  uint32_t lastChange = 0;
  uint8_t clickCount = 0;
  for (;;) {
    bool cur = read_button();
    uint32_t now = millis();
    if (cur != prev && (now - lastChange) > DEBOUNCE_MS) {
      lastChange = now;
      prev = cur;
      if (cur == false) { // pressed edge
        clickCount++;
      }
    }
    if (clickCount > 0 && (now - lastChange) > MULTI_GAP_MS) {
      // classify
      if (clickCount == 1) {
        buzzer_beep(80);
        send_status_report();
        Serial.println("[BTN] single: report sent");
      } else if (clickCount == 2) {
        buzzer_beep(200);
        send_sos();
        Serial.println("[BTN] double: SOS sent");
      } else if (clickCount >= 3) {
        for (int i=0;i<2;i++){ buzzer_beep(60); vTaskDelay(pdMS_TO_TICKS(60)); }
        send_sos_cancel();
        Serial.println("[BTN] triple+: SOS cancel sent");
      }
      clickCount = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== HEARTBEAT TASK ======
static void heartbeat_task(void *arg) {
  for (;;) {
    Serial.printf("[HB] MAC=%s IP=%s RSSI=%d\n", macStr.c_str(), ipToStr(ipAddr).c_str(), WiFi.RSSI());
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// ====== SETUP/LOOP ======
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // avoid brownout resets on spikes

  pinMode(BUTTON_GPIO, INPUT_PULLUP);
  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);

  Serial.begin(115200);
  delay(200);

  wifi_init();
  camera_init();
  start_httpd();

  gpsMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(gps_task,       "gps",       4096, NULL, 2, &hTaskGPS,      1);
  xTaskCreatePinnedToCore(buttons_task,   "buttons",   2048, NULL, 2, &hTaskButtons,  1);
  xTaskCreatePinnedToCore(heartbeat_task, "heartbeat", 2048, NULL, 1, &hTaskHeartbeat,0);

  Serial.println("Endpoints:");
  Serial.println("  /stream   -> MJPEG video");
  Serial.println("  /status   -> JSON status");
  Serial.println("  /gps      -> latest raw NMEA buffer");
}

void loop() {}
