#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESP32SPISlave.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
ESP32SPISlave slave;

// --- SPI Binary Data Structures (16 Bytes) ---
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

TelemetryPacket rxTelemetry;
CommandPacket txCommand = {0}; 
String csvBuffer = ""; // Temporary RAM storage for CSV lines

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  if (!LittleFS.begin(true)) Serial.println("Error mounting LittleFS");

  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI); 

  WiFi.softAP("IPR", "password");
  WiFi.setSleep(false); 
  
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // --- NEW: CSV Download & Clear Endpoints ---
  server.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/log.csv", "text/csv");
  });
  server.on("/clear_log", HTTP_GET, [](AsyncWebServerRequest *request){
    LittleFS.remove("/log.csv");
    request->send(200, "text/plain", "Log Cleared!");
  });

  ws.onEvent(onWsEvent); 
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  static unsigned long lastWsCleanup = 0;
  if (millis() - lastWsCleanup > 1000) { ws.cleanupClients(); lastWsCleanup = millis(); }

  // 1. Queue SPI if buffer is empty
  if (slave.remained() == 0) slave.queue((uint8_t*)&rxTelemetry, (uint8_t*)&txCommand, 16);

  static unsigned long lastSpiReceive = millis();
  static unsigned long lastCsvLog = 0;
  static unsigned long lastWsSend = 0;
  static unsigned long lastFlashWrite = 0;

  // 2. Process Incoming SPI Data
  if (slave.available()) {
    lastSpiReceive = millis();
    txCommand.command = 0; 

    // --- LOG TO RAM BUFFER (100Hz / 10ms) ---
    if (millis() - lastCsvLog >= 10) {
      char line[80];
      snprintf(line, sizeof(line), "%.2f,%.2f,%.2f,%.2f\n", 
               rxTelemetry.pitch, rxTelemetry.angular_velocity, rxTelemetry.setpoint_error, rxTelemetry.front_distance);
      csvBuffer += line;
      lastCsvLog = millis();
    }

    // --- FLUSH TO FLASH (Every 5 Seconds) ---
    if (millis() - lastFlashWrite > 5000 && csvBuffer.length() > 0) {
      File file = LittleFS.open("/log.csv", FILE_APPEND);
      if (file) { file.print(csvBuffer); file.close(); }
      csvBuffer = ""; 
      lastFlashWrite = millis();
    }

    // --- BROADCAST TO WEB UI (10Hz / 100ms) ---
    if (millis() - lastWsSend > 100 && ws.count() > 0) {
      char json[150];
      snprintf(json, sizeof(json), "{\"pitch\":%.2f, \"angular_velocity\":%.2f, \"setpoint_error\":%.2f, \"front_distance\":%.2f}", 
               rxTelemetry.pitch, rxTelemetry.angular_velocity, rxTelemetry.setpoint_error, rxTelemetry.front_distance);
      ws.textAll(json);
      lastWsSend = millis();
    }
    slave.pop(); 
  } 
  
  // 3. SPI Watchdog: Reset bus if sync is lost for 500ms
  else if (millis() - lastSpiReceive > 500) {
    slave.end(); slave.begin(VSPI); lastSpiReceive = millis();
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      JsonDocument doc;
      if (!deserializeJson(doc, data, len)) {
        const char* cmd = doc["command"];
        if (strcmp(cmd, "arm") == 0) txCommand.command = 'A';
        else if (strcmp(cmd, "disarm") == 0) txCommand.command = 'D';
        else if (strcmp(cmd, "estop") == 0) txCommand.command = 'E';
        else if (strcmp(cmd, "reboot") == 0) ESP.restart();
      }
    }
  }
}