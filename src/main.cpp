#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32SPISlave.h>
#include <LittleFS.h>
#include <string.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "tamu_iot";
const char* WIFI_PASS = "";  

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
ESP32SPISlave slave;

struct __attribute__((packed)) TelemetryPacket {
  float pitch_raw;
  float pitch_filtered;
  float angular_velocity;
  float setpoint_error;
  float front_distance;
};

struct __attribute__((packed)) CommandPacket {
  uint8_t command;
  uint8_t padding1[3];
  float kp;
  float ki;
  float kd;
  uint8_t padding2[4];
};

WORD_ALIGNED_ATTR TelemetryPacket rxTelemetry = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
WORD_ALIGNED_ATTR CommandPacket txCommand = {0, {0}, 0.0f, 0.0f, 0.0f, {0}};

unsigned long lastWsCleanup  = 0;
unsigned long lastWsSend     = 0;
unsigned long lastSpiReceive = 0;

float esp_safe(float val) {
    if (isnan(val) || isinf(val)) return 0.0f;
    return val;
}

void clearCommandPacket() {
  txCommand.command = 0;
}

void setCommandPacket(uint8_t cmd) {
  txCommand.command = cmd;
}

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
        txCommand.kp = doc["kp"].as<float>();
        txCommand.ki = doc["ki"].as<float>();
        txCommand.kd = doc["kd"].as<float>();
        Serial.printf("Tuning Received - P:%.3f I:%.3f D:%.3f\n", txCommand.kp, txCommand.ki, txCommand.kd);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  if (!LittleFS.begin(true)) Serial.println("Error mounting LittleFS");

  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI);
  slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 20);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (strlen(WIFI_PASS) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    WiFi.begin(WIFI_SSID); 
  }

  Serial.printf("Connecting to %s", WIFI_SSID);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nFailed to connect, falling back to AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("IPR", "password", 6);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin("ipr")) {
    Serial.println("MDNS responder started");
    MDNS.addService("http", "tcp", 80);
  }

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  clearCommandPacket();
  lastSpiReceive = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastWsCleanup >= 1000) {
    ws.cleanupClients();
    lastWsCleanup = now;
  }

  if (slave.available()) {
    slave.pop();
    lastSpiReceive = now;
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, 20);
  }

  if (now - lastWsSend >= 100 && ws.count() > 0) {
    char json[300]; // Increased buffer size for extra field
    snprintf(json, sizeof(json),
             "{\"pitch\":%.2f,\"pitch_raw\":%.2f,\"angular_velocity\":%.2f,\"setpoint_error\":%.2f,\"front_distance\":%.2f}",
             esp_safe(rxTelemetry.pitch_filtered), 
             esp_safe(rxTelemetry.pitch_raw), // Added raw pitch here
             esp_safe(rxTelemetry.angular_velocity), 
             esp_safe(rxTelemetry.setpoint_error), 
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