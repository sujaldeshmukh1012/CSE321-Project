#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include "esp_camera.h"

// WiFi credentials
const char* ssid = "UB_Devices";
const char* password = "goubbulls";

// Pin definitions
#define GPS_RX 43
#define GPS_TX 44
#define LED_PIN D0
#define BUZZER_PIN D1
#define BUTTON_PIN D2
#define NANO_COMM_PIN D3
#define SDA_PIN 5
#define SCL_PIN 6

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


volatile bool nano_status=false;

void handleRoot();
void handleStream(); 
void handleSOSStatus();
void handleMap();
void handleSendMessage();
void handleButton();
void updateGPSData(); 
void handleCapture();
void updateDisplay();
void setupWebServer();
void connectWiFi();
void initCamera();
// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Default coordinates (if GPS unstable)
float defaultLat = 43.0017;
float defaultLng = -78.7849;

// GPS variables
float latitude = defaultLat;
float longitude = defaultLng;
int satellites = 0;
float altitude = 0;
float speed = 0;

// SOS state
bool sosActive = false;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

// Button handling
unsigned long lastButtonPress = 0;
unsigned long buttonPressStart = 0;
int buttonPressCount = 0;
bool buttonPressed = false;
#define DEBOUNCE_DELAY 50
#define DOUBLE_CLICK_TIME 400

// Web server
WebServer server(80);

// Display message from web
String webMessage = "";

// Camera pins for ESP32-CAM (xiao model settingd)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

void setup() {
  Serial.begin(115200);
  
  // Pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(NANO_COMM_PIN, OUTPUT);
  digitalWrite(NANO_COMM_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // I2C for OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  
  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize camera
  initCamera();
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup web server routes
  setupWebServer();
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Ready!");
  display.println("IP: " + WiFi.localIP().toString());
  display.display();
  delay(2000);
}

void loop() {
  // Handle GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      updateGPSData();
    }
  }
  
  // Handle button presses
  handleButton();
  
  // Handle SOS buzzer
  if (sosActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBuzzerToggle >= 500) {
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      lastBuzzerToggle = currentMillis;
    }
  }
  
  // Handle web server
  server.handleClient();
  
  // Update display
  updateDisplay();
  
  delay(100);
}

void initCamera() {
  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void connectWiFi() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Connecting WiFi...");
  display.display();
  
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/sos_status", handleSOSStatus);
  server.on("/map", handleMap);
  server.on("/send_message", HTTP_POST, handleSendMessage);
  server.on("/capture", handleCapture);
  server.begin();
  Serial.println("Web server started");
  Serial.print("Access at: http://");
  Serial.println(WiFi.localIP());
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Tracker</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:20px;}";
  html += "a{display:block;padding:10px;margin:10px 0;background:#4CAF50;color:white;text-decoration:none;border-radius:5px;}";
  html += "a:hover{background:#45a049;}";
  html += "input,button{padding:10px;margin:5px;}</style></head><body>";
  html += "<h1>ESP32 GPS Tracker</h1>";
  html += "<a href='/stream'>View Live Feed</a>";
  html += "<a href='/sos_status'>SOS Status</a>";
  html += "<a href='/map'>Map View</a>";
  html += "<h3>Send Message to Display</h3>";
  html += "<form action='/send_message' method='POST'>";
  html += "<input type='text' name='message' placeholder='Enter message'>";
  html += "<button type='submit'>Send</button></form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStream() {
  String html = "<!DOCTYPE html><html><head><title>Live Feed</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'></head><body>";
  html += "<h1>Live Camera Feed</h1>";
  html += "<img src='/capture' style='width:100%;max-width:800px;' id='stream'>";
  html += "<script>setInterval(()=>{document.getElementById('stream').src='/capture?'+Date.now()},1000);</script>";
  html += "<br><a href='/'>Back</a></body></html>";
  server.send(200, "text/html", html);
}

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleSOSStatus() {
  String html = "<!DOCTYPE html><html><head><title>SOS Status</title>";
  html += "<meta http-equiv='refresh' content='2'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'></head><body>";
  html += "<h1>SOS Status</h1>";
  html += "<p style='font-size:24px;color:";
  html += sosActive ? "red'>🚨 ACTIVE" : "green'>✓ Inactive";
  html += "</p><p>Lat: " + String(latitude, 6) + "</p>";
  html += "<p>Lng: " + String(longitude, 6) + "</p>";
  html += "<p>Satellites: " + String(satellites) + "</p>";
  html += "<br><a href='/'>Back</a></body></html>";
  server.send(200, "text/html", html);
}

void handleMap() {
  String html = "<!DOCTYPE html><html><head><title>Map View</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>";
  html += "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>";
  html += "<style>body{margin:0;padding:0;}#map{height:100vh;width:100%;}</style></head><body>";
  html += "<div id='map'></div><script>";
  html += "var map=L.map('map').setView([" + String(latitude, 6) + "," + String(longitude, 6) + "],13);";
  html += "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);";
  html += "L.marker([" + String(latitude, 6) + "," + String(longitude, 6) + "]).addTo(map)";
  html += ".bindPopup('Lat: " + String(latitude, 6) + "<br>Lng: " + String(longitude, 6) + "').openPopup();";
  html += "setTimeout(()=>location.reload(),10000);</script></body></html>";
  server.send(200, "text/html", html);
}

void handleSendMessage() {
  if (server.hasArg("message")) {
    webMessage = server.arg("message");
    Serial.println("Message received: " + webMessage);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();
  
  if (reading == LOW && !buttonPressed) {
    if (currentTime - lastButtonPress > DEBOUNCE_DELAY) {
      buttonPressed = true;
      buttonPressStart = currentTime;
      buttonPressCount++;
      
      if (buttonPressCount == 1) {
        lastButtonPress = currentTime;
      }
    }
  } else if (reading == HIGH && buttonPressed) {
    buttonPressed = false;
  }
  
  if (buttonPressCount > 0 && (currentTime - lastButtonPress) > DOUBLE_CLICK_TIME) {
    if (buttonPressCount == 1) {
      if (nano_status){
        nano_status = false;
      digitalWrite(NANO_COMM_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      }else{      // Single press - signal to Nano
      nano_status = true;
      digitalWrite(NANO_COMM_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      }
      Serial.println("Signal sent to Nano");
    } else if (buttonPressCount >= 2) {
      // Double press - toggle SOS
      sosActive = !sosActive;
      if (!sosActive) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
      }
      digitalWrite(LED_PIN, sosActive ? HIGH : LOW);
      Serial.println(sosActive ? "SOS ACTIVATED" : "SOS DEACTIVATED");
    }
    buttonPressCount = 0;
  }
}

void updateGPSData() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    altitude = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    speed = gps.speed.kmph();
  }
  satellites = gps.satellites.value();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  if (webMessage.length() > 0) {
    display.println("MSG: " + webMessage);
    display.println("---");
  }
  
  if (sosActive) {
    display.setTextSize(2);
    display.println("!! SOS !!");
    display.setTextSize(1);
  }
  
  display.print("Lat:");
  display.println(latitude, 5);
  display.print("Lng:");
  display.println(longitude, 5);
  display.print("Sats:");
  display.print(satellites);
  display.print(" Alt:");
  display.println((int)altitude);
  display.print("WiFi:");
  display.println(WiFi.status() == WL_CONNECTED ? "OK" : "X");
  display.print("IP:");
  display.println(WiFi.localIP().toString());
  display.print("Device Status: ");
  display.println(nano_status? "ON" : "OFF"); 
  display.display();
}