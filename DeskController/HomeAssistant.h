// =====================================================================
//  HomeAssistant.h  —  Wi-Fi + MQTT + Home Assistant auto-discovery
//
//  Included at the END of DeskController.ino, so it sees all controller
//  state. Uses the ESP32's built-in MQTT client (esp-mqtt), which ships
//  with Arduino-ESP32 core 3.x. No extra library needed. esp-mqtt runs
//  in its own task and reconnects by itself, so a slow or missing broker
//  never blocks loop(), let alone the control task.
//
//  Topics (base = desk/<HA_DEVICE_ID>):
//    <base>/status  online/offline (retained, last-will)
//    <base>/state   JSON with everything HA shows (retained)
//    <base>/msg     last line the controller printed (retained)
//    <base>/cmd     commands: the same strings as the serial port
//                   ("goto 1050", "p 2", "stop", "home", ...)
//
//  Every HA control just publishes a serial command to <base>/cmd, so
//  HA gets exactly the same checks and safety behaviour as the serial
//  port, and its replies appear in the "Last message" sensor.
// =====================================================================
#pragma once
#include <WiFi.h>
#include "esp_idf_version.h"
#include "mqtt_client.h"

#if ESP_IDF_VERSION_MAJOR < 5
#error "Home Assistant support needs Arduino-ESP32 core 3.x (ESP-IDF 5). Update the board package, or set ENABLE_HOME_ASSISTANT 0."
#endif

// ---------------------------------------------------------------------
static esp_mqtt_client_handle_t haClient = nullptr;
static QueueHandle_t haQueue = nullptr;
static volatile bool haConnected = false, haNeedDiscovery = false;
static bool     haMqttStarted = false, haWifiWasUp = false;
static char     haBase[48], haCmdTopic[64], haStatusTopic[64], haStateTopic[64], haMsgTopic[64];
static char     haLastState[640];
static uint32_t haLastStateMs = 0;

struct HaMsg { char kind; char payload[72]; };   // kind: 'c' command, 'h' HA birth/status

// ---------------------------------------------------------------------
//  MQTT events (run in the esp-mqtt task: just queue things up)
// ---------------------------------------------------------------------
static void haEvent(void*, esp_event_base_t, int32_t id, void* data) {
  esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
  switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
      esp_mqtt_client_subscribe(haClient, haCmdTopic, 1);
      esp_mqtt_client_subscribe(haClient, "homeassistant/status", 1);
      haConnected = true;
      haNeedDiscovery = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      haConnected = false;
      break;
    case MQTT_EVENT_DATA: {
      if (ev->current_data_offset != 0 || ev->data_len != ev->total_data_len) break;   // ignore fragments
      HaMsg m = {};
      if (ev->topic_len == (int)strlen(haCmdTopic) && !strncmp(ev->topic, haCmdTopic, ev->topic_len)) m.kind = 'c';
      else if (ev->topic_len == 20 && !strncmp(ev->topic, "homeassistant/status", 20))              m.kind = 'h';
      else break;
      int n = ev->data_len < (int)sizeof(m.payload) - 1 ? ev->data_len : (int)sizeof(m.payload) - 1;
      memcpy(m.payload, ev->data, n);
      xQueueSend(haQueue, &m, 0);
      break;
    }
    default: break;
  }
}

static void haPublish(const char* topic, const char* payload, bool retain, int qos = 0) {
  if (haClient && haConnected) esp_mqtt_client_publish(haClient, topic, payload, 0, qos, retain);
}

// ---------------------------------------------------------------------
//  Discovery. One retained config message per entity. "~" is HA's
//  base-topic shorthand, expanded to <base> by Home Assistant.
// ---------------------------------------------------------------------
enum HaAvail { AV_ONLINE, AV_HOMED };   // AV_HOMED: unavailable until the desk is homed

static void haDiscover(const char* component, const char* object, HaAvail av, const char* body) {
  static char topic[96], payload[1400];
  snprintf(topic, sizeof topic, "homeassistant/%s/%s/%s/config", component, HA_DEVICE_ID, object);
  const char* avail = av == AV_HOMED
    ? "\"availability\":[{\"topic\":\"~/status\"},"
        "{\"topic\":\"~/state\",\"value_template\":\"{{ 'online' if value_json.homed else 'offline' }}\"}],"
      "\"availability_mode\":\"all\""
    : "\"availability\":[{\"topic\":\"~/status\"}]";
  snprintf(payload, sizeof payload,
    "{\"~\":\"%s\",\"unique_id\":\"%s_%s\",%s,"
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"DIY\","
      "\"model\":\"4-leg ESP32 desk controller\",\"sw_version\":\"%s\"},%s}",
    haBase, HA_DEVICE_ID, object, avail, HA_DEVICE_ID, HA_DEVICE_NAME, FW_VERSION, body);
  haPublish(topic, payload, true, 1);
}

static void haPublishDiscovery() {
  char b[700];
  const float minCm = (MIN_HEIGHT_MM + HEIGHT_OFFSET_MM) / 10.0f;
  const float maxCm = (MAX_HEIGHT_MM + HEIGHT_OFFSET_MM) / 10.0f;

  // ---- main controls (Upsy Desky style) ------------------------------
  // Cover: up / stop / down arrows and a position slider (0% = lowest, 100% = highest)
  snprintf(b, sizeof b,
    "\"name\":null,\"icon\":\"mdi:desk\","
    "\"command_topic\":\"~/cmd\",\"payload_open\":\"up\",\"payload_close\":\"down\",\"payload_stop\":\"stop\","
    "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.cover }}\","
    "\"state_opening\":\"opening\",\"state_closing\":\"closing\",\"state_stopped\":\"stopped\","
    "\"position_topic\":\"~/state\",\"position_template\":\"{{ value_json.position }}\","
    "\"set_position_topic\":\"~/cmd\","
    "\"set_position_template\":\"goto {{ (%.0f + %.0f * position / 100) | round(0) | int }}\"",
    MIN_HEIGHT_MM + HEIGHT_OFFSET_MM, MAX_HEIGHT_MM - MIN_HEIGHT_MM);
  haDiscover("cover", "desk", AV_HOMED, b);

  // Target height slider. Follows the real height when idle.
  snprintf(b, sizeof b,
    "\"name\":\"Target height\",\"icon\":\"mdi:arrow-expand-vertical\","
    "\"command_topic\":\"~/cmd\",\"command_template\":\"goto {{ (value * 10) | round(0) | int }}\","
    "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.target_cm }}\","
    "\"min\":%.1f,\"max\":%.1f,\"step\":0.5,\"mode\":\"slider\","
    "\"unit_of_measurement\":\"cm\",\"device_class\":\"distance\"", minCm, maxCm);
  haDiscover("number", "target_height", AV_HOMED, b);

  haDiscover("sensor", "height", AV_HOMED,
    "\"name\":\"Height\",\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.height_cm }}\","
    "\"unit_of_measurement\":\"cm\",\"device_class\":\"distance\",\"state_class\":\"measurement\","
    "\"suggested_display_precision\":1");

  for (int n = 1; n <= NUM_PRESETS; n++) {
    char obj[24];
    snprintf(obj, sizeof obj, "preset_%d", n);
    snprintf(b, sizeof b,
      "\"name\":\"Preset %d\",\"icon\":\"mdi:numeric-%d-box-outline\","
      "\"command_topic\":\"~/cmd\",\"payload_press\":\"p %d\"", n, n, n);
    haDiscover("button", obj, AV_HOMED, b);

    snprintf(obj, sizeof obj, "save_preset_%d", n);
    snprintf(b, sizeof b,
      "\"name\":\"Save preset %d\",\"icon\":\"mdi:content-save-outline\",\"entity_category\":\"config\","
      "\"command_topic\":\"~/cmd\",\"payload_press\":\"save %d\"", n, n);
    haDiscover("button", obj, AV_HOMED, b);

    // Preset heights are editable directly, without moving the desk (serial: setp)
    snprintf(obj, sizeof obj, "preset_%d_height", n);
    snprintf(b, sizeof b,
      "\"name\":\"Preset %d height\",\"icon\":\"mdi:bookmark-outline\",\"entity_category\":\"config\","
      "\"command_topic\":\"~/cmd\",\"command_template\":\"setp %d {{ (value * 10) | round(0) | int }}\","
      "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.p[%d] }}\","
      "\"min\":%.1f,\"max\":%.1f,\"step\":0.5,\"mode\":\"box\","
      "\"unit_of_measurement\":\"cm\",\"device_class\":\"distance\"", n, n, n - 1, minCm, maxCm);
    haDiscover("number", obj, AV_ONLINE, b);
  }

  haDiscover("button", "stop", AV_ONLINE,
    "\"name\":\"Stop\",\"icon\":\"mdi:stop-circle-outline\",\"command_topic\":\"~/cmd\",\"payload_press\":\"stop\"");
  haDiscover("button", "level", AV_ONLINE,
    "\"name\":\"Level\",\"icon\":\"mdi:align-vertical-center\",\"command_topic\":\"~/cmd\",\"payload_press\":\"level\"");

  // ---- status ------------------------------------------------------
  haDiscover("sensor", "status", AV_ONLINE,
    "\"name\":\"Status\",\"icon\":\"mdi:information-outline\",\"state_topic\":\"~/state\","
    "\"value_template\":\"{{ value_json.status }}\"");
  haDiscover("sensor", "fault", AV_ONLINE,
    "\"name\":\"Fault\",\"icon\":\"mdi:alert-outline\",\"state_topic\":\"~/state\","
    "\"value_template\":\"{{ value_json.fault_msg }}\"");
  haDiscover("binary_sensor", "problem", AV_ONLINE,
    "\"name\":\"Problem\",\"device_class\":\"problem\",\"state_topic\":\"~/state\","
    "\"value_template\":\"{{ 'ON' if value_json.fault else 'OFF' }}\"");
  haDiscover("binary_sensor", "moving", AV_ONLINE,
    "\"name\":\"Moving\",\"device_class\":\"moving\",\"state_topic\":\"~/state\","
    "\"value_template\":\"{{ 'ON' if value_json.cover != 'stopped' else 'OFF' }}\"");

  // ---- maintenance (Configuration section of the device page) ---------
  haDiscover("button", "clear_fault", AV_ONLINE,
    "\"name\":\"Clear fault\",\"icon\":\"mdi:alert-remove-outline\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"clear\"");
  haDiscover("button", "home", AV_ONLINE,
    "\"name\":\"Home\",\"icon\":\"mdi:home-import-outline\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"home\"");
  haDiscover("button", "calibrate", AV_ONLINE,
    "\"name\":\"Calibrate directions\",\"icon\":\"mdi:swap-vertical\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"calib\"");
  haDiscover("button", "calib_up", AV_ONLINE,
    "\"name\":\"Calibration answer: went up\",\"icon\":\"mdi:arrow-up-bold-outline\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"y\"");
  haDiscover("button", "calib_down", AV_ONLINE,
    "\"name\":\"Calibration answer: went down\",\"icon\":\"mdi:arrow-down-bold-outline\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"n\"");
  haDiscover("button", "btncal", AV_ONLINE,
    "\"name\":\"Learn preset buttons\",\"icon\":\"mdi:gesture-tap-button\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"btncal\"");
  haDiscover("button", "restart", AV_ONLINE,
    "\"name\":\"Restart\",\"device_class\":\"restart\",\"entity_category\":\"config\","
    "\"command_topic\":\"~/cmd\",\"payload_press\":\"reboot\"");

  // ---- diagnostics -------------------------------------------------
  haDiscover("sensor", "last_message", AV_ONLINE,
    "\"name\":\"Last message\",\"icon\":\"mdi:message-text-outline\",\"entity_category\":\"diagnostic\","
    "\"state_topic\":\"~/msg\"");
  haDiscover("binary_sensor", "homed", AV_ONLINE,
    "\"name\":\"Homed\",\"icon\":\"mdi:home-check-outline\",\"entity_category\":\"diagnostic\","
    "\"state_topic\":\"~/state\",\"value_template\":\"{{ 'ON' if value_json.homed else 'OFF' }}\"");
  haDiscover("sensor", "sync_error", AV_ONLINE,
    "\"name\":\"Leg sync error\",\"icon\":\"mdi:table-furniture\",\"entity_category\":\"diagnostic\","
    "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.spread_mm }}\","
    "\"unit_of_measurement\":\"mm\",\"state_class\":\"measurement\",\"suggested_display_precision\":1");
  for (int i = 0; i < NUM_LEGS; i++) {
    char obj[16];
    snprintf(obj, sizeof obj, "leg_%d", i);
    snprintf(b, sizeof b,
      "\"name\":\"Leg %d position\",\"entity_category\":\"diagnostic\",\"enabled_by_default\":false,"
      "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.legs[%d] }}\","
      "\"unit_of_measurement\":\"mm\",\"device_class\":\"distance\",\"state_class\":\"measurement\","
      "\"suggested_display_precision\":1", i, i);
    haDiscover("sensor", obj, AV_ONLINE, b);
  }
  haDiscover("sensor", "rssi", AV_ONLINE,
    "\"name\":\"Wi-Fi signal\",\"entity_category\":\"diagnostic\",\"state_topic\":\"~/state\","
    "\"value_template\":\"{{ value_json.rssi }}\",\"unit_of_measurement\":\"dBm\","
    "\"device_class\":\"signal_strength\",\"state_class\":\"measurement\"");
  haDiscover("sensor", "i2c_errors", AV_ONLINE,
    "\"name\":\"Motor driver I2C errors\",\"icon\":\"mdi:lan-disconnect\",\"entity_category\":\"diagnostic\","
    "\"state_topic\":\"~/state\",\"value_template\":\"{{ value_json.i2c_errors }}\",\"state_class\":\"total_increasing\"");
}

// ---------------------------------------------------------------------
//  State JSON (snapshot under the lock, publish outside it)
// ---------------------------------------------------------------------
static bool haBuildState(char* out, size_t n, bool& moving, int& modeKey) {
  int rssi = WiFi.RSSI();
  DeskLock lock;
  float h = meanPos();
  float heightCm = (h + HEIGHT_OFFSET_MM) / 10.0f;
  moving = (mode == M_MOVING || mode == M_HOMING);
  modeKey = mode * 8 + (homed ? 4 : 0) + (btnCalStep >= 0 ? 2 : 0);
  float targetCm = moving && mode == M_MOVING ? (goal + HEIGHT_OFFSET_MM) / 10.0f : heightCm;
  targetCm = roundf(targetCm * 2) / 2;                     // match the slider's 0.5 cm step
  int pos = (int)lroundf((h - MIN_HEIGHT_MM) * 100.0f / (MAX_HEIGHT_MM - MIN_HEIGHT_MM));
  pos = constrain(pos, 0, 100);
  const char* cover = !moving ? "stopped" : (mode == M_HOMING || goal < sp) ? "closing" : "opening";
  const char* status =
    btnCalStep >= 0     ? "learning preset buttons" :
    mode == M_IDLE      ? (homed ? "idle" : "not homed") :
    mode == M_MOVING    ? "moving" :
    mode == M_HOMING    ? "homing" :
    mode == M_FAULT     ? "fault" :
    mode == M_TEST      ? "testing leg" : "calibrating";
  float minP = 1e9f, maxP = -1e9f;
  for (int i = 0; i < NUM_LEGS; i++) { minP = fminf(minP, leg[i].pos); maxP = fmaxf(maxP, leg[i].pos); }

  char faultBuf[120];
  if (mode == M_FAULT) snprintf(faultBuf, sizeof faultBuf, "E%d: %s", faultCode, faultMsg.c_str());
  else                 strcpy(faultBuf, "none");
  for (char* c = faultBuf; *c; c++) if (*c == '"' || *c == '\\') *c = '\'';   // keep the JSON valid

  char p[NUM_PRESETS][12];
  for (int k = 0; k < NUM_PRESETS; k++) {
    if (isnan(presets[k])) strcpy(p[k], "null");
    else snprintf(p[k], sizeof p[k], "%.1f", (presets[k] + HEIGHT_OFFSET_MM) / 10.0f);
  }
  unsigned long i2cErr = 0;
  i2cErr = pca.errors();
  int len = snprintf(out, n,
    "{\"height_cm\":%.1f,\"target_cm\":%.1f,\"position\":%d,\"cover\":\"%s\",\"status\":\"%s\","
    "\"homed\":%s,\"fault\":%d,\"fault_msg\":\"%s\",\"spread_mm\":%.1f,"
    "\"legs\":[%.1f,%.1f,%.1f,%.1f],\"p\":[%s,%s,%s,%s],\"rssi\":%d,\"i2c_errors\":%lu}",
    heightCm, targetCm, pos, cover, status,
    homed ? "true" : "false", mode == M_FAULT ? (int)faultCode : 0, faultBuf, maxP - minP,
    leg[0].pos, leg[1].pos, leg[2].pos, leg[3].pos, p[0], p[1], p[2], p[3],
    rssi, i2cErr);
  return len > 0 && len < (int)n;
}

// ---------------------------------------------------------------------
void haBegin() {
  snprintf(haBase,        sizeof haBase,        "desk/%s", HA_DEVICE_ID);
  snprintf(haCmdTopic,    sizeof haCmdTopic,    "%s/cmd", haBase);
  snprintf(haStatusTopic, sizeof haStatusTopic, "%s/status", haBase);
  snprintf(haStateTopic,  sizeof haStateTopic,  "%s/state", haBase);
  snprintf(haMsgTopic,    sizeof haMsgTopic,    "%s/msg", haBase);
  haQueue = xQueueCreate(8, sizeof(HaMsg));

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HA_DEVICE_ID);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Out.printf("Wi-Fi: connecting to %s...\n", WIFI_SSID);
}

static void haStartMqtt() {
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri                  = MQTT_URI;
  cfg.credentials.username                = MQTT_USER;
  cfg.credentials.authentication.password = MQTT_PASSWORD;
  cfg.credentials.client_id               = HA_DEVICE_ID;
  cfg.session.last_will.topic             = haStatusTopic;
  cfg.session.last_will.msg               = "offline";
  cfg.session.last_will.qos               = 1;
  cfg.session.last_will.retain            = 1;
  cfg.session.keepalive                   = 15;
  cfg.buffer.size                         = 1024;
  cfg.buffer.out_size                     = 2048;
  haClient = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(haClient, MQTT_EVENT_ANY, haEvent, nullptr);
  esp_mqtt_client_start(haClient);
  haMqttStarted = true;
}

void haLoop() {
  uint32_t now = millis();

  // ---- Wi-Fi (reconnects by itself; we just report and start MQTT once)
  static uint32_t lastWifiCheck = 0;
  if (now - lastWifiCheck > 1000) {
    lastWifiCheck = now;
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && !haWifiWasUp) {
      Out.printf("Wi-Fi: connected, IP %s\n", WiFi.localIP().toString().c_str());
      if (!haMqttStarted) haStartMqtt();
    } else if (!up && haWifiWasUp) {
      Out.println("Wi-Fi: disconnected (the desk keeps working locally)");
    }
    haWifiWasUp = up;
  }

  // ---- incoming commands
  HaMsg m;
  while (haQueue && xQueueReceive(haQueue, &m, 0) == pdTRUE) {
    if (m.kind == 'c') handleCommand(String(m.payload));                 // takes the lock itself
    else if (m.kind == 'h' && !strcmp(m.payload, "online")) haNeedDiscovery = true;   // HA restarted
  }

  if (!haConnected) return;

  if (haNeedDiscovery) {
    haNeedDiscovery = false;
    haPublishDiscovery();
    haPublish(haStatusTopic, "online", true, 1);
    haLastState[0] = 0;                                                    // force a state publish
    haLastStateMs = 0;
    Out.println("Home Assistant: connected, discovery published");
  }

  // ---- state: on change, 4 Hz max while moving, 1 Hz max idle, and a
  //      heartbeat every 60 s
  static char buf[sizeof haLastState];
  static uint32_t lastBuild = 0;
  static bool moving = false;
  static int  modeKey = -1, lastModeKey = -1;
  if (now - lastBuild >= 100) {                                          // build at most 10x/s
    lastBuild = now;
    if (haBuildState(buf, sizeof buf, moving, modeKey)) {
      // mode changes (start, stop, fault, homed) go out at once
      uint32_t minGap = modeKey != lastModeKey ? 0 : moving ? 250 : 1000;
      bool changed = strcmp(buf, haLastState) != 0;
      if ((changed && now - haLastStateMs >= minGap) || now - haLastStateMs > 60000) {
        haPublish(haStateTopic, buf, true);
        strcpy(haLastState, buf);
        haLastStateMs = now;
        lastModeKey = modeKey;
      }
    }
  }

  // ---- last printed line -> "Last message" sensor
  char line[200];
  if (Out.takeLastLine(line, sizeof line)) haPublish(haMsgTopic, line, true);
}
