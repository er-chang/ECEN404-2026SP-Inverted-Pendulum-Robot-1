#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32SPISlave.h>
#include <LittleFS.h>
#include <string.h>
#include <ESPmDNS.h>

// const char* WIFI_SSID = "TAMU_IoT";
// const char* WIFI_PASS = "";  

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
ESP32SPISlave slave;

// 1. Updated to match STM32 size (44 bytes)
struct __attribute__((packed)) TelemetryPacket {
  float pitch_raw;
  float pitch_filtered;
  float angular_velocity;

  // 4 Ultrasonic Sensors
  float dist_front;
  float dist_back;
  float dist_left;
  float dist_right;

  float system_state; 

  // 4 Motors
  float pwm_fl;
  float pwm_fr;
  float pwm_bl;
  float pwm_br;

  // Potentiometer
  float raw_pot;
  float pot_ohms;

  float cpu_load;
};

// 2. Updated to include Mode and padded to exactly 44 bytes
struct __attribute__((packed)) CommandPacket {
  uint8_t command;
  uint8_t mode;
  uint8_t padding1[2];
  float kp;
  float ki;
  float kd;
  uint8_t padding2[40]; 
};

WORD_ALIGNED_ATTR TelemetryPacket rxTelemetry = {};
WORD_ALIGNED_ATTR CommandPacket txCommand = {};

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

      if (strcmp(cmd, "arm") == 0) {
          setCommandPacket('A');
      } 
      else if (strcmp(cmd, "estop") == 0) {
          setCommandPacket('E'); 
      } 
      else if (strcmp(cmd, "clear") == 0) {
          clearCommandPacket();
      } 
      else if (strcmp(cmd, "set_mode") == 0) {
          txCommand.command = 'M';
          txCommand.mode = doc["mode"].as<uint8_t>();
          Serial.printf("Mode switched to: %d\n", txCommand.mode);
      } 
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
  slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, sizeof(TelemetryPacket));

  // CONNECTION USING IOT NETWORK
  // WiFi.mode(WIFI_STA);
  // WiFi.setSleep(false);

  // if (strlen(WIFI_PASS) > 0) {
  //   WiFi.begin(WIFI_SSID, WIFI_PASS);
  // } else {
  //   WiFi.begin(WIFI_SSID); 
  // }

  // Serial.printf("Connecting to %s", WIFI_SSID);
  // int timeout = 0;
  // while (WiFi.status() != WL_CONNECTED && timeout < 30) {
  //   delay(500);
  //   Serial.print(".");
  //   timeout++;
  // }

  // if (WiFi.status() == WL_CONNECTED) {
  //   Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
  // } else {
  //   Serial.println("\nFailed to connect, falling back to AP mode");
  //   WiFi.mode(WIFI_AP);
  //   WiFi.softAP("IPR", "password", 6);
  // }

  // CONNECTION USING ESP NETWORK
  // Force the ESP32 to be its own independent network
  WiFi.mode(WIFI_AP);
  
  // set up network name and password
  WiFi.softAP("IPR_Robot", "TAMUIPR1", 6, 0, 4); 

  Serial.println("\nprivate network started");
  Serial.print("Connect to: IPR_Robot\n");
  Serial.print("Password:   TAMUIPR1\n");
  Serial.print("Dashboard:  http://192.168.4.1\n");


  if (MDNS.begin("ipr")) {
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
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, sizeof(TelemetryPacket));
  }

  // STM32 packs system_state into front_distance (0.0 = motors off, 1.0 = armed).
  // Stream telemetry ONLY while motors are armed, so the browser CSV recording
  // captures exactly the arm -> safety-trip window and nothing else.
  static bool prev_motors_on = false;
  bool motors_on = (rxTelemetry.dist_front > 0.5f);
  if (motors_on != prev_motors_on) {
    Serial.printf("[Session] motors %s\n",
                  motors_on ? "ARMED" : "DISARMED");
    prev_motors_on = motors_on;
  }

  // fix: removed '&& motors_on' so the ESP32 always sends data
  if (now - lastWsSend >= 50 && ws.count() > 0) {
    char json[600]; // <-- Increased buffer size
    snprintf(json, sizeof(json),
             // vvv Added the "cpu" tag to the end of this string vvv
             "{\"pitch\":%.2f,\"pitch_raw\":%.2f,\"angular_velocity\":%.2f,\"dist_f\":%.1f,\"dist_b\":%.1f,\"dist_l\":%.1f,\"dist_r\":%.1f,\"pwm_fl\":%.0f,\"pwm_fr\":%.0f,\"pwm_bl\":%.0f,\"pwm_br\":%.0f,\"raw_pot\":%.0f,\"pot_ohms\":%.1f,\"cpu\":%.1f}",
             esp_safe(rxTelemetry.pitch_filtered),
             esp_safe(rxTelemetry.pitch_raw),
             esp_safe(rxTelemetry.angular_velocity),
             esp_safe(rxTelemetry.dist_front),
             esp_safe(rxTelemetry.dist_back),
             esp_safe(rxTelemetry.dist_left),
             esp_safe(rxTelemetry.dist_right),
             esp_safe(rxTelemetry.pwm_fl),
             esp_safe(rxTelemetry.pwm_fr),
             esp_safe(rxTelemetry.pwm_bl),
             esp_safe(rxTelemetry.pwm_br),
             esp_safe(rxTelemetry.raw_pot),
             esp_safe(rxTelemetry.pot_ohms),
             esp_safe(rxTelemetry.cpu_load));
    ws.textAll(json);
    lastWsSend = now;
  }

  if (now - lastSpiReceive > 1000) {
    slave.end();
    slave.begin(VSPI);
    slave.queue((uint8_t *)&rxTelemetry, (uint8_t *)&txCommand, sizeof(TelemetryPacket));
    lastSpiReceive = now;
  }
  vTaskDelay(1);
}