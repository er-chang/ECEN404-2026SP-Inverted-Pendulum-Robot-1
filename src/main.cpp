#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32SPISlave.h>
#include <LittleFS.h>
#include <string.h>
#include <ESPmDNS.h>

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
ESP32SPISlave slave;

// 1. Packet Structures
struct __attribute__((packed)) TelemetryPacket {
  float pitch_raw;
  float pitch_filtered;
  float angular_velocity;
  float setpoint_error;
  float front_distance;
};

// FIX: Added padding1 to align the floats to a 4-byte boundary for the STM32's FPU.
// Adjusted padding2 to maintain the strict 20-byte total size.
struct __attribute__((packed)) CommandPacket {
  uint8_t command;
  uint8_t padding1[3];  // Aligns floats to 4-byte boundary
  float kp;             
  float ki;             
  float kd;             
  uint8_t padding2[4];  // Keeps total size at exactly 20 bytes
};

// 2. Global Variables 
WORD_ALIGNED_ATTR TelemetryPacket rxTelemetry = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
WORD_ALIGNED_ATTR CommandPacket txCommand = {0, {0}, 0.0f, 0.0f, 0.0f, {0}};

// 3. Timing Variables
unsigned long lastWsCleanup  = 0;
unsigned long lastWsSend     = 0;
unsigned long lastSpiReceive = 0;

float esp_safe(float val) {
    if (isnan(val) || isinf(val)) return 0.0f;
    return val;
}

// 4. Command Helper Functions
void clearCommandPacket() {
  txCommand.command = 0;
}

void setCommandPacket(uint8_t cmd) {
  txCommand.command = cmd;
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

      if (strcmp(cmd, "arm") == 0) setCommandPacket('A'); 
      else if (strcmp(cmd, "disarm") == 0 || strcmp(cmd, "estop") == 0) setCommandPacket('E'); 
      else if (strcmp(cmd, "clear") == 0) clearCommandPacket();
      else if (strcmp(cmd, "reboot") == 0) ESP.restart();
      
      else if (strcmp(cmd, "tune") == 0) {
        txCommand.command = 'T'; 
        // FIX: Explicitly cast the JSON variants to floats to prevent implicit conversion failures
        txCommand.kp = doc["kp"].as<float>();
        txCommand.ki = doc["ki"].as<float>();
        txCommand.kd = doc["kd"].as<float>();
        Serial.printf("Tuning Received - P:%.3f I:%.3f D:%.3f\n", txCommand.kp, txCommand.ki, txCommand.kd);
      }
    }
  }
}

// 6. Setup Function
void setup() {
  Serial.begin(115200);

  if (MDNS.begin("ipr")) {
    Serial.println("MDNS responder started");
    MDNS.addService("http", "tcp", 80);
  }
  
  if (!LittleFS.begin(true)) Serial.println("Error mounting LittleFS");

  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI);
  
  slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 20);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP("IPR", "password", 6);
  WiFi.setSleep(false); 

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

  if (now - lastWsCleanup >= 1000) {
    ws.cleanupClients();
    lastWsCleanup = now;
  }

  if (slave.available()) {
    slave.pop(); 
    lastSpiReceive = now;
    
    // FIX: Commented out the auto-clear to prevent the race condition.
    // Let the STM32 read the 'T' or 'A' command multiple times safely. 
    // txCommand.command = 0; 
    
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 20);
  }

  if (now - lastWsSend >= 100 && ws.count() > 0) {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"pitch_raw\":%.2f,\"pitch_filtered\":%.2f,\"angular_velocity\":%.2f,\"setpoint_error\":%.2f,\"front_distance\":%.2f}",
             esp_safe(rxTelemetry.pitch_raw), esp_safe(rxTelemetry.pitch_filtered), 
             esp_safe(rxTelemetry.angular_velocity), esp_safe(rxTelemetry.setpoint_error), 
             esp_safe(rxTelemetry.front_distance));
    ws.textAll(json);
    lastWsSend = now;
  }

  if (now - lastSpiReceive > 1000) {
    slave.end();
    slave.begin(VSPI);
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 20);
    lastSpiReceive = now;
    Serial.println("SPI timeout, restarting slave...");
  }
  vTaskDelay(1); 
}