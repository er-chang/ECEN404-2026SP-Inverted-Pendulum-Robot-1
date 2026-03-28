#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32SPISlave.h>
#include <LittleFS.h>
#include <string.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
ESP32SPISlave slave;

// 1. Packet Structures 
struct __attribute__((packed)) TelemetryPacket {
  float pitch;
  float angular_velocity;
  float setpoint_error;
  float front_distance;
};

struct __attribute__((packed)) CommandPacket {
  uint8_t command;
  uint8_t padding[15];
};

// 2. GLOBAL VARIABLES 
TelemetryPacket rxTelemetry = {0.0f, 0.0f, 0.0f, 0.0f};
CommandPacket txCommand = {0};

// 3. Timing Variables
unsigned long lastWsCleanup  = 0;
unsigned long lastWsSend     = 0;
unsigned long lastSpiReceive = 0;

// Helper to prevent NaN/Inf from breaking JSON parsing on the frontend
float esp_safe(float val) {
    if (isnan(val) || isinf(val)) return 0.0f;
    return val;
}

// 4. Command Helper Functions
void clearCommandPacket() {
  txCommand.command = 0;
  memset(txCommand.padding, 0, sizeof(txCommand.padding));
}

void setCommandPacket(uint8_t cmd) {
  txCommand.command = cmd;
  memset(txCommand.padding, 0, sizeof(txCommand.padding));
}

// 5. WebSocket Event Handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (err) return;

      const char *cmd = doc["command"];
      if (cmd == nullptr) return;

      if (strcmp(cmd, "arm") == 0) {
        setCommandPacket('A'); 
      } else if (strcmp(cmd, "disarm") == 0 || strcmp(cmd, "estop") == 0) {
        setCommandPacket('E'); 
      } else if (strcmp(cmd, "clear") == 0) {
        clearCommandPacket();
      } else if (strcmp(cmd, "reboot") == 0) {
        ESP.restart();
      }
    }
  }
}

// 6. Setup Function
void setup() {
  Serial.begin(115200);
  
  if (!LittleFS.begin(true)) {
    Serial.println("Error mounting LittleFS");
  }

  // Initialize SPI Slave on VSPI pins (23, 19, 18, 5)
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI);
  
  // [FIX] Arm the DMA immediately on boot
  slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 16);

  // Initialize Wi-Fi Access Point
  WiFi.softAP("IPR", "password");
  WiFi.setSleep(false); 

  // Setup Web Server
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  clearCommandPacket();
  lastSpiReceive = millis();
}

// 7. Main Loop
void loop() {
  unsigned long now = millis();

  // Clean up disconnected WebSocket clients
  if (now - lastWsCleanup >= 1000) {
    ws.cleanupClients();
    lastWsCleanup = now;
  }

  // [FIX] Reliable SPI Queueing
  if (slave.available()) {
    slave.pop(); 
    lastSpiReceive = now;
    txCommand.command = 0; 
    
    // [FIX] IMMEDIATELY queue the next transaction to prevent bit-shifting
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 16);
  }

  // [FIX] Broadcast at 10Hz (100ms) to prevent ESPAsyncWebServer memory leaks
  if (now - lastWsSend >= 100 && ws.count() > 0) {
    char json[200];
    snprintf(json, sizeof(json),
             "{\"pitch\":%.2f,\"angular_velocity\":%.2f,\"setpoint_error\":%.2f,\"front_distance\":%.2f}",
             esp_safe(rxTelemetry.pitch), esp_safe(rxTelemetry.angular_velocity), 
             esp_safe(rxTelemetry.setpoint_error), esp_safe(rxTelemetry.front_distance));
    ws.textAll(json);
    lastWsSend = now;
  }

  // [FIX] SPI Watchdog Trap corrected
  if (now - lastSpiReceive > 1000) {
    slave.end();
    slave.begin(VSPI);
    // [FIX] Must re-arm DMA after a reset!
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 16);
    lastSpiReceive = now;
    Serial.println("SPI timeout, restarting slave...");
  }
}