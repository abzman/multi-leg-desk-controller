// =====================================================================
//  test_wifi_mqtt  —  Wi-Fi and the MQTT broker on their own
//
//  Uses the same secrets.h and the same built-in ESP32 MQTT client as
//  the firmware, but publishes nothing Home Assistant will pick up. It
//  only uses the topic desk/<HA_DEVICE_ID>/test.
//
//  Setup: copy secrets.h from the main sketch folder into this folder.
//  Needs Arduino-ESP32 core 3.x.
//
//  It connects to Wi-Fi (prints IP and signal), then the broker
//  (credentials check), subscribes to its own test topic and publishes
//  a hello. If you see "echo received", the broker round trip works.
//
//  Serial 115200:
//    pub <text>     publish to desk/<id>/test (you should see the echo)
//    status         Wi-Fi / MQTT status
//
//  Watch from a PC:  mosquitto_sub -h <broker> -u <user> -P <pass> -t 'desk/#' -v
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include <WiFi.h>
#include "esp_idf_version.h"
#include "mqtt_client.h"
#include "Config.h"
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "Copy secrets.h (from secrets.example.h in the main sketch) into this test folder"
#endif

esp_mqtt_client_handle_t client = nullptr;
volatile bool connected = false, gotEcho = false;
char topic[64];
uint32_t sentAt = 0;

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}

static void onEvent(void*, esp_event_base_t, int32_t id, void* data) {
  esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
  switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
      connected = true;
      esp_mqtt_client_subscribe(client, topic, 1);
      Serial.printf("MQTT: connected to %s as '%s'\n", MQTT_URI, MQTT_USER);
      esp_mqtt_client_publish(client, topic, "hello from the desk controller test", 0, 1, 0);
      sentAt = millis();
      break;
    case MQTT_EVENT_DISCONNECTED:
      if (connected) Serial.println("MQTT: disconnected (will retry)");
      connected = false;
      break;
    case MQTT_EVENT_DATA:
      Serial.printf("MQTT: echo received on %.*s: '%.*s'  (round trip %lu ms)\n", ev->topic_len, ev->topic,
                    ev->data_len, ev->data, (unsigned long)(millis() - sentAt));
      gotEcho = true;
      break;
    case MQTT_EVENT_ERROR:
      Serial.println("MQTT: error. Wrong URI/port, broker down, or bad user/password?");
      break;
    default: break;
  }
}

void status() {
  Serial.printf("Wi-Fi: %s", WiFi.status() == WL_CONNECTED ? "connected" : "NOT connected");
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf(", IP %s, signal %d dBm%s", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
                  WiFi.RSSI() < -75 ? " (weak: expect drop-outs)" : "");
  Serial.printf("\nMQTT: %s, echo %s\n", connected ? "connected" : "not connected", gotEcho ? "OK" : "not seen yet");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_wifi_mqtt ===");
  snprintf(topic, sizeof topic, "desk/%s/test", HA_DEVICE_ID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HA_DEVICE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Wi-Fi: connecting to '%s'", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) { Serial.println("!! Wi-Fi failed after 20 s: check SSID/password, 2.4 GHz only"); return; }
  status();

  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri                  = MQTT_URI;
  cfg.credentials.username                = MQTT_USER;
  cfg.credentials.authentication.password = MQTT_PASSWORD;
  cfg.credentials.client_id               = HA_DEVICE_ID "-test";
  client = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, onEvent, nullptr);
  esp_mqtt_client_start(client);
  Serial.printf("MQTT: connecting to %s ...\n", MQTT_URI);
}

void loop() {
  String line = readLine();
  if (line.startsWith("pub ") && client) {
    sentAt = millis();
    esp_mqtt_client_publish(client, topic, line.c_str() + 4, 0, 1, 0);
    Serial.printf("published '%s'\n", line.c_str() + 4);
  }
  else if (line == "status") status();
  else if (line.length()) Serial.println("? pub <text> | status");

  static bool warned = false;
  if (!warned && client && !connected && millis() > 20000) {
    warned = true;
    Serial.println("!! Still no MQTT connection. Check MQTT_URI (mqtt://IP:1883), the user/password, and that the Mosquitto add-on is running.");
  }
}
