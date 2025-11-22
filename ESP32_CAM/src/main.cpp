#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "esp_camera.h"

// ====================================================================
// --- WiFi Credentials ---
// ====================================================================
const char* ssid = "UB_Devices";
const char* password = "goubbulls";

// BUZZER_PIN has been removed as requested.
#define BUTTON_PIN 3        // D2/GPIO3 (Digital Input)
#define BLUE_LED_PIN 45     // D8/GPIO45 (Digital Output - SOS Indicator)

// OLED Display (I2C)
// Uses D4 (GPIO4) for SDA and D5 (GPIO5) for SCL
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// The Wire library automatically uses the pins specified in Wire.begin()
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// GPS & Serial
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);        // UART1 for GPS: RX=44(D7), TX=43(D6)
HardwareSerial nanoSerial(2);       // UART2 for Nano: RX=2(D1), TX=1(D0)

// Web Server
WebServer server(80);

// ====================================================================
// --- Global Variables ---
// ====================================================================
bool sosActive = false;
bool anomalyDetected = false;
float latitude = 0.0;
float longitude = 0.0;
int satellites = 0;

// FreeRTOS task handles
TaskHandle_t gpsTaskHandle;
TaskHandle_t nanoCommTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t buttonTaskHandle;


camera_config_t camera_config = {
  .pin_pwdn = -1,
  .pin_reset = -1,
  .pin_xclk = 10,
  .pin_sscb_sda = 40,   // FIXED: Reverted to original GPIO 40
  .pin_sscb_scl = 39,   // FIXED: Reverted to original GPIO 39
  .pin_d7 = 48,
  .pin_d6 = 11,
  .pin_d5 = 12,
  .pin_d4 = 14,
  .pin_d3 = 16,
  .pin_d2 = 18,
  .pin_d1 = 17,
  .pin_d0 = 15,
  .pin_vsync = 38,
  .pin_href = 47,
  .pin_pclk = 13,
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_SVGA,
  .jpeg_quality = 12,
  .fb_count = 1
};

void handleRoot();
void handleGPS();
void handleSOS();
void handleCamera();
void handleStatus();
void handleStream();
void gpsTask(void *parameter);
void nanoCommTask(void *parameter);
void displayTask(void *parameter);
void buttonTask(void *parameter);


void handleRoot() {
  // Beautified HTML response for the main dashboard
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Caretaker Dashboard</title>";
  html += "<style>";
  // Responsive and clean styling
  html += "body { font-family: 'Inter', sans-serif; background-color: #1a1a2e; color: #fff; margin: 0; padding: 20px; text-align: center; }";
  html += "h1 { color: #00bcd4; margin-bottom: 30px; font-size: 1.8em; }";
  html += ".container { max-width: 400px; margin: 0 auto; background-color: #2c2c4d; padding: 25px; border-radius: 12px; box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5); }";
  html += ".button-grid { display: grid; grid-template-columns: 1fr; gap: 15px; margin-top: 20px; }";
  html += ".button { display: block; padding: 15px; text-decoration: none; color: #fff; border-radius: 8px; font-weight: bold; transition: background-color 0.3s, transform 0.1s; box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2); }";
  html += ".button:hover { transform: translateY(-1px); box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3); }";
  html += ".status { margin-top: 10px; margin-bottom: 25px; font-size: 0.9em; color: #a0a0ff; }";

  // Specific button colors
  html += ".link-data { background-color: #3f51b5; }";
  html += ".link-data:hover { background-color: #303f9f; }";
  html += ".link-image { background-color: #ff9800; }";
  html += ".link-image:hover { background-color: #f57c00; }";
  html += ".link-stream { background-color: #e91e63; }";
  html += ".link-stream:hover { background-color: #c2185b; }";

  html += "</style>";
  html += "</head><body><div class='container'>";
  html += "<h1>Caretaker Visually Impared Assistant</h1>";
  html += "<p class='status'>Microcontroller Live Dashboard & API Access</p>";
  html += "<div class='button-grid'>";
  
  // Grouping JSON endpoints
  html += "<a href='/gps' class='button link-data'>1. GPS Data (JSON)</a>";
  html += "<a href='/sos' class='button link-data'>2. SOS/Anomaly Status (JSON)</a>";
  html += "<a href='/status' class='button link-data'>3. Device Status (JSON)</a>";
  
  // Camera endpoints
  html += "<a href='/camera' class='button link-image'>4. View Single Image (JPEG)</a>";
  html += "<a href='/stream' class='button link-stream'>5. Start Live Video Stream</a>";
  
  html += "</div></div></body></html>";
  server.send(200, "text/html", html);
}

void handleGPS() {
  String json = "{";
  json += "\"latitude\":" + String(latitude, 6) + ",";
  json += "\"longitude\":" + String(longitude, 6) + ",";
  json += "\"satellites\":" + String(satellites) + ",";
  json += "\"timestamp\":" + String(millis());
  json += "}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleSOS() {
  String json = "{";
  json += "\"sos_active\":" + String(sosActive ? "true" : "false") + ",";
  json += "\"anomaly_detected\":" + String(anomalyDetected ? "true" : "false") + ",";
  json += "\"latitude\":" + String(latitude, 6) + ",";
  json += "\"longitude\":" + String(longitude, 6) + ",";
  json += "\"timestamp\":" + String(millis());
  json += "}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleCamera() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  server.client().write_P((const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleStream() {
  WiFiClient client = server.client();
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n";
  response += "Access-Control-Allow-Origin: *\r\n";
  response += "Connection: keep-alive\r\n\r\n";
  
  client.write((const uint8_t *)response.c_str(), response.length());
  
  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(100);
      continue;
    }
    
    String part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    part += String(fb->len);
    part += "\r\n\r\n";
    
    client.write((const uint8_t *)part.c_str(), part.length());
    client.write(fb->buf, fb->len);
    client.write((const uint8_t *)"\r\n", 2);
    
    esp_camera_fb_return(fb);
    
    delay(1); 
  }
}

void handleStatus() {
  String json = "{";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"gps_fix\":" + String(satellites > 0 ? "true" : "false") + ",";
  json += "\"sos_active\":" + String(sosActive ? "true" : "false") + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap());
  json += "}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void gpsTask(void *parameter) {
  while(1) {
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
    }
    
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      satellites = gps.satellites.value();
      
      Serial.printf("GPS: Lat=%.6f, Lon=%.6f, Sats=%d\n", latitude, longitude, satellites);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void nanoCommTask(void *parameter) {
  String receivedData = "";
  
  while(1) {
    while (nanoSerial.available() > 0) {
      char c = nanoSerial.read();
      if (c == '\n') {
        if (receivedData.startsWith("ANOMALY:")) {
          anomalyDetected = true;
          sosActive = true; // Activate SOS
          Serial.println("ANOMALY DETECTED FROM NANO! SOS activated.");
        }
        receivedData = "";
      } else {
        receivedData += c;
      }
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void displayTask(void *parameter) {
  while(1) {
    if(sosActive) {
      digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN)); 
    } else {
      digitalWrite(BLUE_LED_PIN, LOW);
    }


    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    if(WiFi.status() == WL_CONNECTED) {
      display.print("WiFi:OK ");
      display.println(WiFi.RSSI());
    } else {
      display.println("WiFi:DISCONN");
    }

    display.print("GPS:");
    if(satellites > 0) {
      display.print(satellites);
      display.println(" sats");
    } else {
      display.println("No fix");
    }

    if(latitude != 0.0 && longitude != 0.0) {
      display.print(latitude, 4);
      display.print(",");
      display.println(longitude, 4);
    }
    
    if(sosActive) {
      display.setTextSize(2);
      display.println("SOS!");
      display.setTextSize(1);
      display.println("ALERT ACTIVE");
    } else if(anomalyDetected) {
      display.println("Anomaly Cleared");
    } else {
      display.println("Status: OK");
    }
    
    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void buttonTask(void *parameter) {
  bool lastButtonState = HIGH;
  unsigned long lastDebounce = 0;
  
  while(1) {
    bool buttonState = digitalRead(BUTTON_PIN);
    
    if(buttonState != lastButtonState) {
      lastDebounce = millis();
    }
    
    if((millis() - lastDebounce) > 50) {
      if(buttonState == LOW && lastButtonState == HIGH) {
        sosActive = !sosActive;
        anomalyDetected = sosActive;
        
        if(sosActive) {
          Serial.println("SOS Activated by button");
        } else {
          Serial.println("SOS Deactivated");
        }
      
        vTaskDelay(300 / portTICK_PERIOD_MS); 
      }
    }
    
    lastButtonState = buttonState;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\nBooting VioSense Cam (XIAO S3)...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  Wire.begin(4, 5);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    display.println("Camera FAIL!");
    display.display();
    delay(2000);
  } else {
    Serial.println("Camera initialized");
  }

  gpsSerial.begin(9600, SERIAL_8N1, 44, 43);

  nanoSerial.begin(115200, SERIAL_8N1, 2, 1);
  
  WiFi.begin(ssid, password);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected");
    display.print("IP:");
    display.println(WiFi.localIP());
    display.display();
  } else {
    Serial.println("WiFi failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi FAILED!");
    display.display();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/gps", HTTP_GET, handleGPS);
  server.on("/sos", HTTP_GET, handleSOS);
  server.on("/camera", HTTP_GET, handleCamera);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/status", HTTP_GET, handleStatus);
  
  server.begin();
  Serial.println("HTTP server started");
  
  delay(1000);

  xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 1, &gpsTaskHandle, 0);
  xTaskCreatePinnedToCore(nanoCommTask, "Nano Comm", 4096, NULL, 2, &nanoCommTaskHandle, 0);
  xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, &displayTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonTask, "Button Task", 2048, NULL, 1, &buttonTaskHandle, 1);
  
  Serial.println("Setup complete!");
}

void loop() {
  server.handleClient();
  delay(10);
}