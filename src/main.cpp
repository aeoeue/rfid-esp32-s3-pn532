#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <vector>
#include <functional>
#include <esp_ota_ops.h>
#include <esp_err.h>
#include "tag_reader.h"
#include "door_controller.h"

static const char* FW_VERSION = "0.5.0";
static const char* BUILD_STAMP = __DATE__ " " __TIME__;
static const uint16_t CONFIG_SCHEMA_VERSION = 2;
static const char* PREF_NAMESPACE = "rfid";
static const char* TAGS_FILE = "/tags.json";
static const size_t LOG_MAX_LINES = 120;
static const size_t EVENT_MAX_ITEMS = 40;

struct AppConfig {
  String deviceName = "rfid-esp32";
  String wifiSsid;
  String wifiPassword;
  String apPassword = "configureme";
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPassword;
  String mqttTopicPrefix = "rfid";
  bool mqttDiscovery = true;
  int tzOffsetMin = 0;
  bool scheduleEnabled = false;
  String scheduleDays = "1,2,3,4,5,6,0";
  String scheduleStart = "00:00";
  String scheduleEnd = "23:59";
  uint32_t cooldownMs = 1500;
  bool outputEnabled = false;
  int outputPin = 4;
  bool outputActiveHigh = true;
  uint32_t outputPulseMs = 800;
  String outputIdleMode = "low";
  uint32_t outputMaxPulseMs = 5000;
  uint8_t outputLockoutLimit = 3;
  uint32_t outputLockoutWindowMs = 15000;
  uint32_t outputLockoutMs = 10000;
  int pinSck = 12;
  int pinMiso = 13;
  int pinMosi = 11;
  int pinSs = 10;
  int pinRst = -1;
  uint32_t readerPollMs = 40;
  uint32_t readerRemoveMs = 700;
  uint32_t readerSuppressMs = 250;
};

enum class TagState : uint8_t {
  Allowed = 0,
  Blocked = 1,
  Disabled = 2,
};

struct TagInfo {
  String uid;
  String name;
  TagState state = TagState::Allowed;
  String action = "none";
  uint32_t scans = 0;
  uint32_t lastSeenMs = 0;
  uint32_t lastDecisionMs = 0;
};

struct RecentEvent {
  uint32_t timestampMs = 0;
  String uid;
  String name;
  String event;
  bool published = false;
};

static AppConfig g_cfg;
static std::vector<TagInfo> g_tags;
static std::vector<RecentEvent> g_recentEvents;
static Preferences g_prefs;
static WebServer g_server(80);
static WiFiClient g_wifiClient;
static PubSubClient g_mqtt(g_wifiClient);
static ITagReader* g_reader = nullptr;
static DoorController g_door;
static DNSServer g_dns;

static bool g_learningMode = false;
static bool g_learningNextOnly = false;
static String g_learningName;
static uint32_t g_lastMqttReconnectTry = 0;
static uint32_t g_lastWiFiTry = 0;
static uint32_t g_lastStatePublishMs = 0;
static uint32_t g_lastEventsSaveMs = 0;
static uint32_t g_lastDiagnosticsLogMs = 0;
static String g_lastUid;
static String g_lastName;
static String g_lastEvent;
static String g_deviceId;
static String g_baseTopic;
static String g_resetReason;
static std::vector<String> g_logBuffer;
static bool g_fsAvailable = false;
static bool g_apActive = false;
static bool g_dnsActive = false;
static bool g_mdnsStarted = false;
static bool g_eventsDirty = false;
static wl_status_t g_prevWiFiStatus = WL_IDLE_STATUS;
static int g_prevMqttState = -100;
static ReaderState g_prevReaderState = ReaderState::Uninitialized;
static bool g_prevDoorLogicalState = false;
static String g_prevDoorStatus;
static uint16_t g_loadedConfigSchema = 0;

static void addLog(const String& msg) {
  String line = String("[") + String(millis()) + "ms] " + msg;
  Serial.println(line);
  g_logBuffer.push_back(line);
  if (g_logBuffer.size() > LOG_MAX_LINES) {
    g_logBuffer.erase(g_logBuffer.begin());
  }
}

static String getChipId() {
  uint64_t chipid = ESP.getEfuseMac();
  char out[17];
  snprintf(out, sizeof(out), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(out);
}

static String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static String stateToString(TagState state) {
  switch (state) {
    case TagState::Allowed: return "allowed";
    case TagState::Blocked: return "blocked";
    case TagState::Disabled: return "disabled";
  }
  return "allowed";
}

static TagState stringToState(const String& s) {
  if (s == "blocked") return TagState::Blocked;
  if (s == "disabled") return TagState::Disabled;
  return TagState::Allowed;
}

static bool isValidTagAction(const String& action) {
  return action == "none" || action == "pulse" || action == "toggle";
}

static bool isValidIdleMode(const String& mode) {
  return mode == "low" || mode == "high" || mode == "pullup" || mode == "pulldown" || mode == "floating";
}

static String formatIsoTime(time_t ts) {
  if (ts < 1700000000) return "";
  struct tm tmUtc;
  gmtime_r(&ts, &tmUtc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
  return String(buf);
}

static String currentIsoTime() {
  return formatIsoTime(time(nullptr));
}

static String partitionSummary(const esp_partition_t* part) {
  if (part == nullptr) return "none";
  String out = part->label;
  out += " subtype=" + String((int)part->subtype);
  out += " addr=0x" + String((uint32_t)part->address, HEX);
  return out;
}

static String otaImageStateToString(const esp_partition_t* part) {
  if (part == nullptr) return "unknown";
  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(part, &state);
  if (err != ESP_OK) return "unknown";
  switch (state) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    case ESP_OTA_IMG_UNDEFINED:
    default:
      return "undefined";
  }
}

static void ensureOtaStateValid() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* boot = esp_ota_get_boot_partition();
  addLog(String("OTA running=") + partitionSummary(running));
  addLog(String("OTA boot=") + partitionSummary(boot));
  addLog(String("OTA running_state=") + otaImageStateToString(running));
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    addLog("OTA app marked valid");
  } else if (err == ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
    addLog("OTA app already valid");
  } else {
    addLog(String("OTA mark valid failed err=") + String((int)err));
  }
}

static void markEventsDirty() {
  g_eventsDirty = true;
}

static void saveRecentEventsNow() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("events");
  for (const auto& ev : g_recentEvents) {
    JsonObject o = arr.createNestedObject();
    o["timestamp_ms"] = ev.timestampMs;
    o["uid"] = ev.uid;
    o["name"] = ev.name;
    o["event"] = ev.event;
    o["published"] = ev.published;
  }
  String payload;
  serializeJson(doc, payload);
  g_prefs.putString("events_json", payload);
  g_lastEventsSaveMs = millis();
  g_eventsDirty = false;
}

static void loadRecentEvents() {
  g_recentEvents.clear();
  String json = g_prefs.getString("events_json", "");
  if (json.isEmpty()) return;
  DynamicJsonDocument doc(8192);
  auto err = deserializeJson(doc, json);
  if (err) {
    addLog(String("Failed to parse events_json: ") + err.c_str());
    return;
  }
  JsonArray arr = doc["events"].as<JsonArray>();
  for (JsonVariant v : arr) {
    RecentEvent ev;
    ev.timestampMs = v["timestamp_ms"] | 0;
    ev.uid = v["uid"] | "";
    ev.name = v["name"] | "";
    ev.event = v["event"] | "";
    ev.published = v["published"] | false;
    if (!ev.uid.isEmpty() || !ev.event.isEmpty()) {
      g_recentEvents.push_back(ev);
    }
  }
}

static void queueRecentEvent(const String& uid, const String& name, const String& eventType) {
  RecentEvent ev;
  ev.timestampMs = millis();
  ev.uid = uid;
  ev.name = name;
  ev.event = eventType;
  ev.published = false;
  g_recentEvents.push_back(ev);
  if (g_recentEvents.size() > EVENT_MAX_ITEMS) {
    g_recentEvents.erase(g_recentEvents.begin());
  }
  markEventsDirty();
}

static TagInfo* findTag(const String& uid) {
  for (auto& tag : g_tags) {
    if (tag.uid == uid) return &tag;
  }
  return nullptr;
}

static int parseHHMM(const String& s) {
  if (s.length() != 5 || s.charAt(2) != ':') return -1;
  int hh = s.substring(0, 2).toInt();
  int mm = s.substring(3, 5).toInt();
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return -1;
  return hh * 60 + mm;
}

static bool dayAllowed(int wday) {
  return g_cfg.scheduleDays.indexOf(String(wday)) >= 0;
}

static bool scheduleAllowsNow() {
  if (!g_cfg.scheduleEnabled) return true;
  time_t now = time(nullptr);
  if (now < 1700000000) return true;
  struct tm tmNow;
  localtime_r(&now, &tmNow);
  if (!dayAllowed(tmNow.tm_wday)) return false;
  int from = parseHHMM(g_cfg.scheduleStart);
  int to = parseHHMM(g_cfg.scheduleEnd);
  if (from < 0 || to < 0) return true;
  int nowMin = tmNow.tm_hour * 60 + tmNow.tm_min;
  if (from <= to) return nowMin >= from && nowMin <= to;
  return nowMin >= from || nowMin <= to;
}

static void saveConfig() {
  g_prefs.putUShort("schema", CONFIG_SCHEMA_VERSION);
  g_prefs.putString("device", g_cfg.deviceName);
  g_prefs.putString("w_ssid", g_cfg.wifiSsid);
  g_prefs.putString("w_pass", g_cfg.wifiPassword);
  g_prefs.putString("ap_pass", g_cfg.apPassword);
  g_prefs.putString("m_host", g_cfg.mqttHost);
  g_prefs.putUShort("m_port", g_cfg.mqttPort);
  g_prefs.putString("m_user", g_cfg.mqttUser);
  g_prefs.putString("m_pass", g_cfg.mqttPassword);
  g_prefs.putString("m_pref", g_cfg.mqttTopicPrefix);
  g_prefs.putBool("m_disc", g_cfg.mqttDiscovery);
  g_prefs.putInt("tz_off", g_cfg.tzOffsetMin);
  g_prefs.putBool("sched_en", g_cfg.scheduleEnabled);
  g_prefs.putString("sched_d", g_cfg.scheduleDays);
  g_prefs.putString("sched_s", g_cfg.scheduleStart);
  g_prefs.putString("sched_e", g_cfg.scheduleEnd);
  g_prefs.putUInt("cool_ms", g_cfg.cooldownMs);
  g_prefs.putBool("o_en", g_cfg.outputEnabled);
  g_prefs.putInt("o_pin", g_cfg.outputPin);
  g_prefs.putBool("o_act_h", g_cfg.outputActiveHigh);
  g_prefs.putUInt("o_pulse", g_cfg.outputPulseMs);
  g_prefs.putString("o_idle", g_cfg.outputIdleMode);
  g_prefs.putUInt("o_pmax", g_cfg.outputMaxPulseMs);
  g_prefs.putUChar("o_lock_n", g_cfg.outputLockoutLimit);
  g_prefs.putUInt("o_lock_w", g_cfg.outputLockoutWindowMs);
  g_prefs.putUInt("o_lock_ms", g_cfg.outputLockoutMs);
  g_prefs.putInt("p_sck", g_cfg.pinSck);
  g_prefs.putInt("p_miso", g_cfg.pinMiso);
  g_prefs.putInt("p_mosi", g_cfg.pinMosi);
  g_prefs.putInt("p_ss", g_cfg.pinSs);
  g_prefs.putInt("p_rst", g_cfg.pinRst);
  g_prefs.putUInt("r_poll", g_cfg.readerPollMs);
  g_prefs.putUInt("r_rm", g_cfg.readerRemoveMs);
  g_prefs.putUInt("r_sup", g_cfg.readerSuppressMs);
}

static void loadConfig() {
  g_loadedConfigSchema = g_prefs.getUShort("schema", 0);
  if (g_loadedConfigSchema == 0) {
    addLog("Config schema: legacy, applying defaults for new fields");
  } else {
    addLog(String("Config schema loaded: ") + String(g_loadedConfigSchema));
  }
  g_cfg.deviceName = g_prefs.getString("device", g_cfg.deviceName);
  g_cfg.wifiSsid = g_prefs.getString("w_ssid", "");
  g_cfg.wifiPassword = g_prefs.getString("w_pass", "");
  g_cfg.apPassword = g_prefs.getString("ap_pass", g_cfg.apPassword);
  g_cfg.mqttHost = g_prefs.getString("m_host", "");
  g_cfg.mqttPort = g_prefs.getUShort("m_port", g_cfg.mqttPort);
  g_cfg.mqttUser = g_prefs.getString("m_user", "");
  g_cfg.mqttPassword = g_prefs.getString("m_pass", "");
  g_cfg.mqttTopicPrefix = g_prefs.getString("m_pref", g_cfg.mqttTopicPrefix);
  g_cfg.mqttDiscovery = g_prefs.getBool("m_disc", g_cfg.mqttDiscovery);
  g_cfg.tzOffsetMin = g_prefs.getInt("tz_off", g_cfg.tzOffsetMin);
  g_cfg.scheduleEnabled = g_prefs.getBool("sched_en", g_cfg.scheduleEnabled);
  g_cfg.scheduleDays = g_prefs.getString("sched_d", g_cfg.scheduleDays);
  g_cfg.scheduleStart = g_prefs.getString("sched_s", g_cfg.scheduleStart);
  g_cfg.scheduleEnd = g_prefs.getString("sched_e", g_cfg.scheduleEnd);
  g_cfg.cooldownMs = g_prefs.getUInt("cool_ms", g_cfg.cooldownMs);
  g_cfg.outputEnabled = g_prefs.getBool("o_en", g_cfg.outputEnabled);
  g_cfg.outputPin = g_prefs.getInt("o_pin", g_cfg.outputPin);
  g_cfg.outputActiveHigh = g_prefs.getBool("o_act_h", g_cfg.outputActiveHigh);
  g_cfg.outputPulseMs = g_prefs.getUInt("o_pulse", g_cfg.outputPulseMs);
  g_cfg.outputIdleMode = g_prefs.getString("o_idle", g_cfg.outputIdleMode);
  g_cfg.outputMaxPulseMs = g_prefs.getUInt("o_pmax", g_cfg.outputMaxPulseMs);
  g_cfg.outputLockoutLimit = g_prefs.getUChar("o_lock_n", g_cfg.outputLockoutLimit);
  g_cfg.outputLockoutWindowMs = g_prefs.getUInt("o_lock_w", g_cfg.outputLockoutWindowMs);
  g_cfg.outputLockoutMs = g_prefs.getUInt("o_lock_ms", g_cfg.outputLockoutMs);
  g_cfg.pinSck = g_prefs.getInt("p_sck", g_cfg.pinSck);
  g_cfg.pinMiso = g_prefs.getInt("p_miso", g_cfg.pinMiso);
  g_cfg.pinMosi = g_prefs.getInt("p_mosi", g_cfg.pinMosi);
  g_cfg.pinSs = g_prefs.getInt("p_ss", g_cfg.pinSs);
  g_cfg.pinRst = g_prefs.getInt("p_rst", g_cfg.pinRst);
  g_cfg.readerPollMs = g_prefs.getUInt("r_poll", g_cfg.readerPollMs);
  g_cfg.readerRemoveMs = g_prefs.getUInt("r_rm", g_cfg.readerRemoveMs);
  g_cfg.readerSuppressMs = g_prefs.getUInt("r_sup", g_cfg.readerSuppressMs);
  if (g_cfg.apPassword.length() > 0 && g_cfg.apPassword.length() < 8) {
    g_cfg.apPassword = "configureme";
  }
  if (!isValidIdleMode(g_cfg.outputIdleMode)) {
    g_cfg.outputIdleMode = "low";
  }
}

static void saveTags() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("tags");
  for (const auto& tag : g_tags) {
    JsonObject o = arr.createNestedObject();
    o["uid"] = tag.uid;
    o["name"] = tag.name;
    o["state"] = stateToString(tag.state);
    o["action"] = tag.action;
  }

  String serialized;
  serializeJson(doc, serialized);

  bool fileOk = false;
  if (g_fsAvailable) {
    File f = LittleFS.open(TAGS_FILE, "w");
    if (f) {
      f.print(serialized);
      f.close();
      fileOk = true;
    } else {
      addLog("Failed to open tags file for write");
    }
  }

  if (!fileOk) {
    g_prefs.putString("tags_json", serialized);
    addLog("Tags persisted to NVS fallback");
  }
}

static void loadTags() {
  g_tags.clear();
  DynamicJsonDocument doc(8192);
  DeserializationError err;

  if (g_fsAvailable && LittleFS.exists(TAGS_FILE)) {
    File f = LittleFS.open(TAGS_FILE, "r");
    if (!f) return;
    err = deserializeJson(doc, f);
    f.close();
  } else {
    String json = g_prefs.getString("tags_json", "");
    if (json.isEmpty()) return;
    err = deserializeJson(doc, json);
  }

  if (err) {
    addLog(String("Failed to parse tags.json: ") + err.c_str());
    return;
  }

  JsonArray arr = doc["tags"].as<JsonArray>();
  for (JsonVariant v : arr) {
    TagInfo tag;
    tag.uid = v["uid"].as<String>();
    tag.name = v["name"].as<String>();
    tag.state = stringToState(v["state"].as<String>());
    tag.action = v["action"] | "none";
    if (!isValidTagAction(tag.action)) tag.action = "none";
    if (!tag.uid.isEmpty()) {
      g_tags.push_back(tag);
    }
  }
}

static String tagsToJson() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("tags");
  for (const auto& tag : g_tags) {
    JsonObject o = arr.createNestedObject();
    o["uid"] = tag.uid;
    o["name"] = tag.name;
    o["state"] = stateToString(tag.state);
    o["action"] = tag.action;
    o["scans"] = tag.scans;
    o["last_seen_ms"] = tag.lastSeenMs;
    o["last_decision_ms"] = tag.lastDecisionMs;
  }
  doc["count"] = (int)g_tags.size();
  String out;
  serializeJson(doc, out);
  return out;
}

static String eventsToJson() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("events");
  for (const auto& ev : g_recentEvents) {
    JsonObject o = arr.createNestedObject();
    o["timestamp_ms"] = ev.timestampMs;
    o["uid"] = ev.uid;
    o["name"] = ev.name;
    o["event"] = ev.event;
    o["published"] = ev.published;
  }
  doc["count"] = (int)g_recentEvents.size();
  String out;
  serializeJson(doc, out);
  return out;
}

static bool mqttPublish(const String& topic, const String& payload, bool retained = false) {
  if (!g_mqtt.connected()) {
    addLog(String("MQTT publish skipped (not connected): ") + topic);
    return false;
  }
  bool ok = g_mqtt.publish(topic.c_str(), payload.c_str(), retained);
  if (!ok) {
    addLog(String("MQTT publish failed topic=") + topic + " len=" + String(payload.length()));
  }
  return ok;
}

static void publishLearningState() {
  mqttPublish(g_baseTopic + "/learning/state", g_learningMode ? "ON" : "OFF", true);
}

static void publishOutputState() {
  mqttPublish(g_baseTopic + "/output/state", g_door.currentLogicalState() ? "ON" : "OFF", true);
}

static bool publishRecentEventAt(size_t index) {
  if (index >= g_recentEvents.size()) return false;
  const RecentEvent& ev = g_recentEvents[index];
  DynamicJsonDocument doc(512);
  doc["uid"] = ev.uid;
  doc["name"] = ev.name;
  doc["event"] = ev.event;
  doc["timestamp_ms"] = ev.timestampMs;
  String iso = currentIsoTime();
  if (!iso.isEmpty()) doc["time"] = iso;
  TagInfo* tag = findTag(ev.uid);
  if (tag != nullptr) {
    doc["state"] = stateToString(tag->state);
  }
  String payload;
  serializeJson(doc, payload);
  bool ok = mqttPublish(g_baseTopic + "/event", payload, false);
  if (ok && !g_recentEvents[index].published) {
    g_recentEvents[index].published = true;
    markEventsDirty();
  }
  return ok;
}

static void publishPendingEvents() {
  bool anyChanged = false;
  for (size_t i = 0; i < g_recentEvents.size(); ++i) {
    if (!g_recentEvents[i].published && publishRecentEventAt(i)) {
      anyChanged = true;
    }
  }
  if (anyChanged) markEventsDirty();
}

static void publishState() {
  DynamicJsonDocument doc(2048);
  doc["device"] = g_cfg.deviceName;
  doc["chip_id"] = g_deviceId;
  doc["fw_version"] = FW_VERSION;
  doc["build_stamp"] = BUILD_STAMP;
  doc["uptime_s"] = millis() / 1000;
  doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  doc["wifi_ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("");
  doc["wifi_rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["mqtt_connected"] = g_mqtt.connected();
  doc["mqtt_state_code"] = g_mqtt.state();
  doc["learning"] = g_learningMode;
  doc["learning_next_only"] = g_learningNextOnly;
  doc["tags_count"] = (int)g_tags.size();
  doc["events_count"] = (int)g_recentEvents.size();
  doc["schedule_enabled"] = g_cfg.scheduleEnabled;
  doc["output_enabled"] = g_cfg.outputEnabled;
  doc["output_state"] = g_door.currentLogicalState();
  doc["output_status"] = g_door.statusText();
  doc["output_lockout"] = g_door.isLockedOut();
  doc["output_pulse_remaining_ms"] = g_door.pulseRemainingMs();
  doc["output_lockout_remaining_ms"] = g_door.lockoutRemainingMs();
  doc["reader_name"] = g_reader ? g_reader->readerName() : String("none");
  doc["reader_state"] = g_reader ? readerStateToString(g_reader->state()) : "none";
  doc["reader_status"] = g_reader ? g_reader->statusText() : String("missing");
  doc["reader_uid"] = g_reader ? g_reader->currentUid() : String("");
  doc["reader_uid_age_ms"] = g_reader ? g_reader->currentUidAgeMs() : 0;
  doc["reader_error"] = g_reader ? g_reader->lastErrorCode() : -1;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["reset_reason"] = g_resetReason;
  doc["last_uid"] = g_lastUid;
  doc["last_name"] = g_lastName;
  doc["last_event"] = g_lastEvent;

  String payload;
  serializeJson(doc, payload);
  mqttPublish(g_baseTopic + "/status", payload, true);
  mqttPublish(g_baseTopic + "/sensor/last_uid", g_lastUid, true);
  mqttPublish(g_baseTopic + "/sensor/last_name", g_lastName, true);
  mqttPublish(g_baseTopic + "/sensor/last_event", g_lastEvent, true);
  mqttPublish(g_baseTopic + "/sensor/tags_count", String((int)g_tags.size()), true);
  mqttPublish(g_baseTopic + "/sensor/wifi_rssi", String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0), true);
  mqttPublish(g_baseTopic + "/sensor/free_heap", String(ESP.getFreeHeap()), true);
  mqttPublish(g_baseTopic + "/sensor/reader_state", g_reader ? readerStateToString(g_reader->state()) : "none", true);
  mqttPublish(g_baseTopic + "/sensor/door_status", g_door.statusText(), true);
}

static void publishDiscovery() {
  if (!g_cfg.mqttDiscovery) return;
  String discoveryPrefix = "homeassistant";

  auto sendEntity = [&](const String& comp, const String& obj, const std::function<void(JsonObject)>& fill) {
    DynamicJsonDocument d(1024);
    JsonObject root = d.to<JsonObject>();
    root["uniq_id"] = g_deviceId + "_" + obj;
    root["name"] = g_cfg.deviceName + " " + obj;
    root["avty_t"] = g_baseTopic + "/availability";
    root["pl_avail"] = "online";
    root["pl_not_avail"] = "offline";
    JsonObject dev = root.createNestedObject("dev");
    dev["name"] = g_cfg.deviceName;
    JsonArray ids = dev.createNestedArray("ids");
    ids.add(g_deviceId);
    dev["mf"] = "Custom";
    dev["mdl"] = "ESP32-S3 PN532";
    dev["sw"] = FW_VERSION;
    fill(root);

    String topic = discoveryPrefix + "/" + comp + "/" + g_deviceId + "/" + obj + "/config";
    String payload;
    serializeJson(d, payload);
    bool ok = mqttPublish(topic, payload, true);
    addLog(String("HA discovery ") + (ok ? "published" : "failed") + " for " + comp + "/" + obj);
  };

  sendEntity("sensor", "last_uid", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/last_uid"; });
  sendEntity("sensor", "last_name", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/last_name"; });
  sendEntity("sensor", "last_event", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/last_event"; });
  sendEntity("sensor", "tags_count", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/tags_count"; });
  sendEntity("sensor", "wifi_rssi", [&](JsonObject o) {
    o["stat_t"] = g_baseTopic + "/sensor/wifi_rssi";
    o["unit_of_meas"] = "dBm";
  });
  sendEntity("sensor", "free_heap", [&](JsonObject o) {
    o["stat_t"] = g_baseTopic + "/sensor/free_heap";
    o["unit_of_meas"] = "B";
  });
  sendEntity("sensor", "reader_state", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/reader_state"; });
  sendEntity("sensor", "door_status", [&](JsonObject o) { o["stat_t"] = g_baseTopic + "/sensor/door_status"; });
  sendEntity("switch", "learning", [&](JsonObject o) {
    o["stat_t"] = g_baseTopic + "/learning/state";
    o["cmd_t"] = g_baseTopic + "/learning/set";
    o["pl_on"] = "ON";
    o["pl_off"] = "OFF";
  });
  sendEntity("button", "learn_next", [&](JsonObject o) {
    o["cmd_t"] = g_baseTopic + "/learn_next/set";
    o["pl_prs"] = "PRESS";
  });
  sendEntity("button", "reboot", [&](JsonObject o) {
    o["cmd_t"] = g_baseTopic + "/reboot/set";
    o["pl_prs"] = "PRESS";
  });
  sendEntity("switch", "output", [&](JsonObject o) {
    o["stat_t"] = g_baseTopic + "/output/state";
    o["cmd_t"] = g_baseTopic + "/output/set";
    o["pl_on"] = "ON";
    o["pl_off"] = "OFF";
  });
  sendEntity("button", "pulse_output", [&](JsonObject o) {
    o["cmd_t"] = g_baseTopic + "/pulse_output/set";
    o["pl_prs"] = "PRESS";
  });
}

static void blinkStatusLed(int count, int onMs, int offMs) {
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < count; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(offMs);
  }
#else
  (void)count;
  (void)onMs;
  (void)offMs;
#endif
}

static void configureDoorController() {
  DoorControllerConfig cfg;
  cfg.enabled = g_cfg.outputEnabled;
  cfg.pin = g_cfg.outputPin;
  cfg.activeHigh = g_cfg.outputActiveHigh;
  cfg.pulseMs = g_cfg.outputPulseMs;
  cfg.idleMode = g_cfg.outputIdleMode;
  cfg.maxPulseMs = g_cfg.outputMaxPulseMs;
  cfg.lockoutLimit = g_cfg.outputLockoutLimit;
  cfg.lockoutWindowMs = g_cfg.outputLockoutWindowMs;
  cfg.lockoutMs = g_cfg.outputLockoutMs;
  g_door.configure(cfg);
  g_prevDoorLogicalState = g_door.currentLogicalState();
  g_prevDoorStatus = g_door.statusText();
  addLog(String("Door controller configured pin=") + String(g_cfg.outputPin) + " idle=" + g_cfg.outputIdleMode + " enabled=" + (g_cfg.outputEnabled ? "true" : "false"));
}

static void restartReader() {
  if (g_reader != nullptr) {
    delete g_reader;
    g_reader = nullptr;
  }
  g_reader = createPn532SpiReader();
  ReaderConfig cfg;
  cfg.pinSck = g_cfg.pinSck;
  cfg.pinMiso = g_cfg.pinMiso;
  cfg.pinMosi = g_cfg.pinMosi;
  cfg.pinSs = g_cfg.pinSs;
  cfg.pinRst = g_cfg.pinRst;
  cfg.pollIntervalMs = g_cfg.readerPollMs;
  cfg.removeAfterMs = g_cfg.readerRemoveMs;
  cfg.duplicateSuppressMs = g_cfg.readerSuppressMs;
  bool ok = g_reader->begin(cfg);
  g_prevReaderState = g_reader->state();
  if (ok) {
    addLog(String("Reader initialized: ") + g_reader->readerName() + " status=" + g_reader->statusText());
  } else {
    addLog(String("Reader init failed: state=") + readerStateToString(g_reader->state()) + " err=" + String(g_reader->lastErrorCode()));
  }
}

static void rebuildBaseTopic() {
  g_baseTopic = g_cfg.mqttTopicPrefix + "/" + g_cfg.deviceName;
}

static void restartMdnsIfNeeded() {
  if (g_mdnsStarted) {
    MDNS.end();
    g_mdnsStarted = false;
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin(g_cfg.deviceName.c_str())) {
      g_mdnsStarted = true;
      addLog(String("mDNS started: http://") + g_cfg.deviceName + ".local");
    } else {
      addLog("mDNS start failed");
    }
  }
}

static bool requestOutputPulse(const String& source) {
  String reason;
  bool ok = g_door.requestPulse(&reason);
  if (!ok) {
    addLog(String("Output pulse rejected from ") + source + ": " + reason);
  } else {
    addLog(String("Output pulse triggered from ") + source);
  }
  publishOutputState();
  publishState();
  return ok;
}

static bool setOutputManual(bool on, const String& source) {
  String reason;
  bool ok = g_door.setManual(on, &reason);
  if (!ok) {
    addLog(String("Output set rejected from ") + source + ": " + reason);
  } else {
    addLog(String("Output ") + (on ? "ON" : "OFF") + " from " + source);
  }
  publishOutputState();
  publishState();
  return ok;
}

static void runTagAction(TagInfo* tag) {
  if (tag == nullptr) return;
  if (tag->action == "pulse") {
    requestOutputPulse("tag");
  } else if (tag->action == "toggle") {
    setOutputManual(!g_door.currentLogicalState(), "tag");
  }
}

static void connectWifiIfNeeded(bool force = false) {
  if (g_cfg.wifiSsid.isEmpty()) return;
  if (!force && millis() - g_lastWiFiTry < 10000) return;
  if (WiFi.status() == WL_CONNECTED) return;
  g_lastWiFiTry = millis();
  addLog(String("WiFi connect attempt SSID=") + g_cfg.wifiSsid);
  WiFi.begin(g_cfg.wifiSsid.c_str(), g_cfg.wifiPassword.c_str());
}

static void ensureAccessPointIfNeeded() {
  bool noStaConfig = g_cfg.wifiSsid.isEmpty();
  bool staDown = WiFi.status() != WL_CONNECTED;
  if (!(noStaConfig || staDown)) {
    if (g_apActive) {
      WiFi.softAPdisconnect(true);
      g_apActive = false;
      if (g_dnsActive) {
        g_dns.stop();
        g_dnsActive = false;
      }
      addLog("Setup AP stopped (STA connected)");
    }
    return;
  }

  String apSsid = "RFID-Setup-" + g_deviceId.substring(g_deviceId.length() - 4);
  bool ok = g_cfg.apPassword.length() >= 8 ? WiFi.softAP(apSsid.c_str(), g_cfg.apPassword.c_str()) : WiFi.softAP(apSsid.c_str());
  if (ok && !g_apActive) {
    g_apActive = true;
    addLog(String("AP active: ") + apSsid + " IP=" + WiFi.softAPIP().toString());
    g_dns.start(53, "*", WiFi.softAPIP());
    g_dnsActive = true;
    addLog("Captive DNS started");
  }
}

static void connectMqttIfNeeded();
static void handleCommandJson(const JsonObject& cmd);
static void publishState();

static void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  String tpc(topic);
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) msg += (char)payload[i];

  if (tpc == g_baseTopic + "/learning/set") {
    if (msg == "ON") {
      g_learningMode = true;
      g_learningNextOnly = false;
    } else if (msg == "OFF") {
      g_learningMode = false;
      g_learningNextOnly = false;
      g_learningName = "";
    }
    publishLearningState();
    publishState();
    return;
  }

  if (tpc == g_baseTopic + "/learn_next/set") {
    g_learningMode = true;
    g_learningNextOnly = true;
    g_learningName = "";
    publishLearningState();
    publishState();
    return;
  }

  if (tpc == g_baseTopic + "/reboot/set") {
    addLog("Reboot requested via HA button");
    delay(200);
    ESP.restart();
    return;
  }

  if (tpc == g_baseTopic + "/pulse_output/set") {
    requestOutputPulse("mqtt_button");
    return;
  }

  if (tpc == g_baseTopic + "/output/set") {
    if (msg == "ON") {
      setOutputManual(true, "mqtt_switch");
    } else if (msg == "OFF") {
      setOutputManual(false, "mqtt_switch");
    }
    return;
  }

  if (tpc == g_baseTopic + "/cmd") {
    DynamicJsonDocument doc(1024);
    auto err = deserializeJson(doc, msg);
    if (!err) handleCommandJson(doc.as<JsonObject>());
  }
}

static void connectMqttIfNeeded() {
  if (g_cfg.mqttHost.isEmpty()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (g_mqtt.connected()) return;
  if (millis() - g_lastMqttReconnectTry < 5000) return;
  g_lastMqttReconnectTry = millis();

  g_mqtt.setServer(g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  g_mqtt.setCallback(mqttCallback);
  g_mqtt.setBufferSize(2048);
  addLog(String("MQTT connect attempt to ") + g_cfg.mqttHost + ":" + String(g_cfg.mqttPort));

  String willTopic = g_baseTopic + "/availability";
  bool ok;
  if (g_cfg.mqttUser.isEmpty()) {
    ok = g_mqtt.connect(g_deviceId.c_str(), willTopic.c_str(), 0, true, "offline");
  } else {
    ok = g_mqtt.connect(g_deviceId.c_str(), g_cfg.mqttUser.c_str(), g_cfg.mqttPassword.c_str(), willTopic.c_str(), 0, true, "offline");
  }

  if (!ok) {
    addLog(String("MQTT connect failed, state=") + String(g_mqtt.state()));
    return;
  }

  addLog("MQTT connected");
  mqttPublish(g_baseTopic + "/availability", "online", true);
  g_mqtt.subscribe((g_baseTopic + "/cmd").c_str());
  g_mqtt.subscribe((g_baseTopic + "/learning/set").c_str());
  g_mqtt.subscribe((g_baseTopic + "/learn_next/set").c_str());
  g_mqtt.subscribe((g_baseTopic + "/reboot/set").c_str());
  g_mqtt.subscribe((g_baseTopic + "/output/set").c_str());
  g_mqtt.subscribe((g_baseTopic + "/pulse_output/set").c_str());
  addLog(String("MQTT subscribed on base topic ") + g_baseTopic);
  publishDiscovery();
  publishLearningState();
  publishOutputState();
  publishPendingEvents();
  publishState();
}

static void handleCommandJson(const JsonObject& cmd) {
  String action = cmd["action"] | "";
  String uid = cmd["uid"] | "";
  uid.toUpperCase();

  if (action == "learn_next") {
    g_learningMode = true;
    g_learningNextOnly = true;
    g_learningName = cmd["name"] | "";
    publishLearningState();
    publishState();
    return;
  }
  if (action == "set_learning") {
    g_learningMode = cmd["enabled"].as<bool>();
    g_learningNextOnly = cmd["next_only"].as<bool>();
    g_learningName = cmd["name"] | "";
    publishLearningState();
    publishState();
    return;
  }
  if (action == "set_tag_state") {
    TagInfo* tag = findTag(uid);
    if (!tag) return;
    tag->state = stringToState(cmd["state"] | "allowed");
    saveTags();
    publishState();
    return;
  }
  if (action == "rename_tag") {
    TagInfo* tag = findTag(uid);
    if (!tag) return;
    tag->name = cmd["name"] | tag->name;
    saveTags();
    publishState();
    return;
  }
  if (action == "set_tag_action") {
    TagInfo* tag = findTag(uid);
    if (!tag) return;
    String tagAction = cmd["tag_action"] | "none";
    if (!isValidTagAction(tagAction)) return;
    tag->action = tagAction;
    saveTags();
    publishState();
    return;
  }
  if (action == "delete_tag") {
    for (size_t i = 0; i < g_tags.size(); ++i) {
      if (g_tags[i].uid == uid) {
        g_tags.erase(g_tags.begin() + i);
        break;
      }
    }
    saveTags();
    publishState();
    return;
  }
  if (action == "list_tags") {
    mqttPublish(g_baseTopic + "/response", tagsToJson(), false);
    return;
  }
  if (action == "pulse_output") {
    requestOutputPulse("mqtt_cmd");
    return;
  }
  if (action == "reboot") {
    addLog("Reboot requested via MQTT command");
    delay(200);
    ESP.restart();
  }
}

static void applyRuntimeConfigChanges(bool wifiChanged, bool mqttChanged, bool topicChanged, bool readerChanged, bool outputChanged, bool deviceNameChanged, bool apChanged) {
  if (topicChanged || deviceNameChanged) {
    rebuildBaseTopic();
    mqttChanged = true;
    addLog(String("MQTT base topic updated to ") + g_baseTopic);
  }
  if (mqttChanged && g_mqtt.connected()) {
    mqttPublish(g_baseTopic + "/availability", "offline", true);
    g_mqtt.disconnect();
    addLog("MQTT disconnected for config reload");
  }
  if (outputChanged) {
    configureDoorController();
    publishOutputState();
  }
  if (readerChanged) {
    restartReader();
  }
  if (wifiChanged) {
    WiFi.disconnect(true, true);
    delay(100);
    connectWifiIfNeeded(true);
  }
  if (apChanged || wifiChanged) {
    ensureAccessPointIfNeeded();
  }
  if (deviceNameChanged) {
    WiFi.setHostname(g_cfg.deviceName.c_str());
    restartMdnsIfNeeded();
  }
  configTime(g_cfg.tzOffsetMin * 60, 0, "pool.ntp.org", "time.nist.gov");
  publishState();
}

static bool validateBackup(const JsonDocument& doc, String& error) {
  JsonObjectConst cfg = doc["config"].as<JsonObjectConst>();
  if (cfg.isNull()) {
    error = "backup missing config";
    return false;
  }
  if (!cfg["apPassword"].isNull()) {
    String ap = cfg["apPassword"].as<String>();
    if (!ap.isEmpty() && ap.length() < 8) {
      error = "apPassword must be empty or >=8 chars";
      return false;
    }
  }
  if (!cfg["outputIdleMode"].isNull()) {
    String mode = cfg["outputIdleMode"].as<String>();
    if (!isValidIdleMode(mode)) {
      error = "invalid outputIdleMode";
      return false;
    }
  }
  JsonArrayConst tags = doc["tags"].as<JsonArrayConst>();
  std::vector<String> seen;
  for (JsonVariantConst v : tags) {
    String uid = v["uid"] | "";
    String action = v["action"] | "none";
    if (uid.isEmpty()) {
      error = "backup contains empty tag uid";
      return false;
    }
    for (const auto& existing : seen) {
      if (existing == uid) {
        error = "backup contains duplicate tag uid";
        return false;
      }
    }
    seen.push_back(uid);
    if (!isValidTagAction(action)) {
      error = "backup contains invalid tag action";
      return false;
    }
    String state = v["state"] | "allowed";
    if (!(state == "allowed" || state == "blocked" || state == "disabled")) {
      error = "backup contains invalid tag state";
      return false;
    }
  }
  return true;
}

static void sendJson(int code, const String& body) {
  g_server.send(code, "application/json", body);
}

static String readBody() {
  return g_server.hasArg("plain") ? g_server.arg("plain") : String("");
}

static const char INDEX_HTML[] PROGMEM = R"html(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>RFID Controller</title>
  <style>
    :root {
      --bg: #eef4ee;
      --card: #ffffff;
      --ink: #152018;
      --muted: #66756a;
      --accent: #1b7f4d;
      --accent-dark: #284636;
      --warn: #b43333;
      --line: #d6e2d8;
      --good: #e1f6e7;
      --bad: #f8e2e2;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Trebuchet MS", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top right, #dbf4dc, transparent 34%),
        radial-gradient(circle at 14% 18%, #fdf1cb, transparent 22%),
        var(--bg);
    }
    .wrap { max-width: 1120px; margin: 0 auto; padding: 16px; }
    .card {
      background: var(--card);
      border-radius: 14px;
      padding: 14px;
      margin-bottom: 12px;
      box-shadow: 0 8px 24px rgba(0,0,0,.06);
    }
    h1, h2, h3 { margin-top: 0; }
    h2 { margin-bottom: 8px; font-size: 18px; }
    h3 { margin-bottom: 6px; font-size: 15px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
    .grid3 { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
    @media (max-width: 860px) {
      .grid, .grid3 { grid-template-columns: 1fr; }
    }
    label { display: block; margin-top: 8px; margin-bottom: 4px; font-weight: bold; font-size: 14px; }
    input, button, select, textarea {
      width: 100%;
      padding: 9px 10px;
      border-radius: 9px;
      border: 1px solid #c9d7cb;
      background: #fff;
      color: var(--ink);
      font: inherit;
    }
    button {
      width: auto;
      background: var(--accent);
      color: white;
      border: none;
      cursor: pointer;
      margin-right: 6px;
      margin-top: 8px;
    }
    button.secondary { background: var(--accent-dark); }
    button.danger { background: var(--warn); }
    .inline { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    .inline > * { width: auto; }
    .banner { display: none; padding: 10px 12px; border-radius: 10px; margin-bottom: 12px; }
    .banner.ok { display: block; background: var(--good); }
    .banner.err { display: block; background: var(--bad); }
    pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      background: #f6faf6;
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 10px;
      max-height: 320px;
      overflow: auto;
    }
    table { width: 100%; border-collapse: collapse; }
    th, td {
      padding: 8px 4px;
      border-bottom: 1px solid var(--line);
      text-align: left;
      vertical-align: top;
      font-size: 14px;
    }
    .muted { color: var(--muted); }
    .pill {
      display: inline-block;
      padding: 3px 8px;
      border-radius: 999px;
      background: #edf4ee;
      font-size: 12px;
      margin-right: 6px;
      margin-bottom: 4px;
    }
    .help ul { margin: 8px 0 0 18px; padding: 0; }
    .mini { font-size: 13px; }
  </style>
</head>
<body>
<div class="wrap">
  <h1>RFID Controller</h1>
  <div id="banner" class="banner"></div>

  <div class="grid">
    <div class="card">
      <h2>Status</h2>
      <div class="inline">
        <span id="wifiPill" class="pill">wifi</span>
        <span id="mqttPill" class="pill">mqtt</span>
        <span id="readerPill" class="pill">reader</span>
        <span id="doorPill" class="pill">door</span>
      </div>
      <pre id="status">loading...</pre>
      <div class="inline">
        <button onclick="pulseOutput()">Pulse Output</button>
        <button class="secondary" onclick="outputOff()">Force Idle</button>
        <button class="danger" onclick="rebootNow()">Reboot Device</button>
      </div>
    </div>
    <div class="card">
      <h2>Diagnostics</h2>
      <pre id="diag">loading...</pre>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Learning</h2>
      <label for="learnName">Name for next tag</label>
      <input id="learnName" />
      <div class="inline">
        <button onclick="setLearning(true,false)">Enable Learning</button>
        <button class="secondary" onclick="learnNext()">Learn Next Tag</button>
        <button class="secondary" onclick="setLearning(false,false)">Disable Learning</button>
      </div>
      <p class="muted mini">Use learning mode to store unknown tags. "Learn next" turns learning off after the next scan.</p>
    </div>
    <div class="card help">
      <h2>Wiring Help</h2>
      <div class="mini">
        <strong>PN532 SPI</strong>
        <ul>
          <li>ESP32 GPIO12 -> PN532 SCK</li>
          <li>ESP32 GPIO13 -> PN532 MISO</li>
          <li>ESP32 GPIO11 -> PN532 MOSI</li>
          <li>ESP32 GPIO10 -> PN532 SS / NSS / SDA</li>
          <li>ESP32 3V3 -> PN532 VCC</li>
          <li>ESP32 GND -> PN532 GND</li>
          <li>Optional reset pin -> PN532 RST / RSTPDN</li>
        </ul>
        <strong>Door Output</strong>
        <ul>
          <li>Use a relay or proper driver board. Do not power the lock directly from the ESP pin.</li>
          <li>Idle mode defines the safe level when the output is inactive.</li>
          <li>Lockout prevents repeated trigger bursts from hammering the door hardware.</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>Tags</h2>
    <div class="inline">
      <button onclick="loadTags()">Refresh Tags</button>
      <button class="secondary" onclick="loadEvents()">Refresh Events</button>
    </div>
    <table id="tagsTbl"></table>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Recent Events</h2>
      <pre id="events">loading...</pre>
    </div>
    <div class="card">
      <h2>Debug Log</h2>
      <button class="secondary" onclick="loadLogs()">Refresh Logs</button>
      <pre id="logs">loading...</pre>
    </div>
  </div>

  <div class="card">
    <h2>Configuration</h2>
    <div class="grid3">
      <div>
        <h3>Network</h3>
        <label for="deviceName">Device Name</label>
        <input id="deviceName" />
        <label for="wifiSsid">WiFi SSID</label>
        <input id="wifiSsid" />
        <label for="wifiPassword">WiFi Password</label>
        <input id="wifiPassword" type="password" />
        <label for="apPassword">Setup AP Password</label>
        <input id="apPassword" type="password" />
        <label for="mqttHost">MQTT Host / IP</label>
        <input id="mqttHost" />
        <label for="mqttPort">MQTT Port</label>
        <input id="mqttPort" type="number" />
        <label for="mqttUser">MQTT User</label>
        <input id="mqttUser" />
        <label for="mqttPassword">MQTT Password</label>
        <input id="mqttPassword" type="password" />
        <label for="mqttTopicPrefix">MQTT Topic Prefix</label>
        <input id="mqttTopicPrefix" />
        <label for="tzOffsetMin">Timezone Offset Minutes</label>
        <input id="tzOffsetMin" type="number" />
        <label><input id="mqttDiscovery" type="checkbox" /> Home Assistant Discovery</label>
      </div>
      <div>
        <h3>Reader</h3>
        <label for="pinSck">PN532 SCK Pin</label>
        <input id="pinSck" type="number" />
        <label for="pinMiso">PN532 MISO Pin</label>
        <input id="pinMiso" type="number" />
        <label for="pinMosi">PN532 MOSI Pin</label>
        <input id="pinMosi" type="number" />
        <label for="pinSs">PN532 SS Pin</label>
        <input id="pinSs" type="number" />
        <label for="pinRst">PN532 Reset Pin</label>
        <input id="pinRst" type="number" />
        <label for="readerPollMs">Reader Poll Interval (ms)</label>
        <input id="readerPollMs" type="number" />
        <label for="readerRemoveMs">Tag Removed Timeout (ms)</label>
        <input id="readerRemoveMs" type="number" />
        <label for="readerSuppressMs">Duplicate Suppress (ms)</label>
        <input id="readerSuppressMs" type="number" />
        <h3>Access Rules</h3>
        <label><input id="scheduleEnabled" type="checkbox" /> Schedule Enabled</label>
        <label for="scheduleDays">Allowed Days (0=Sun..6=Sat)</label>
        <input id="scheduleDays" />
        <label for="scheduleStart">Schedule Start (HH:MM)</label>
        <input id="scheduleStart" />
        <label for="scheduleEnd">Schedule End (HH:MM)</label>
        <input id="scheduleEnd" />
        <label for="cooldownMs">Tag Cooldown (ms)</label>
        <input id="cooldownMs" type="number" />
      </div>
      <div>
        <h3>Door Output</h3>
        <label><input id="outputEnabled" type="checkbox" /> Output Enabled</label>
        <label for="outputPin">Output Pin</label>
        <input id="outputPin" type="number" />
        <label><input id="outputActiveHigh" type="checkbox" /> Active High</label>
        <label for="outputPulseMs">Pulse Duration (ms)</label>
        <input id="outputPulseMs" type="number" />
        <label for="outputMaxPulseMs">Maximum Pulse Duration (ms)</label>
        <input id="outputMaxPulseMs" type="number" />
        <label for="outputIdleMode">Idle Mode</label>
        <select id="outputIdleMode">
          <option value="low">Low</option>
          <option value="high">High</option>
          <option value="pullup">Pull-up</option>
          <option value="pulldown">Pull-down</option>
          <option value="floating">Floating</option>
        </select>
        <label for="outputLockoutLimit">Lockout Trigger Count</label>
        <input id="outputLockoutLimit" type="number" />
        <label for="outputLockoutWindowMs">Lockout Window (ms)</label>
        <input id="outputLockoutWindowMs" type="number" />
        <label for="outputLockoutMs">Lockout Duration (ms)</label>
        <input id="outputLockoutMs" type="number" />
      </div>
    </div>
    <div class="inline">
      <button onclick="saveConfig()">Save Configuration</button>
    </div>
    <p class="muted mini">Configuration changes are applied immediately. Reader, WiFi and MQTT changes may briefly disconnect the device while it reinitializes.</p>
  </div>

  <div class="grid">
    <div class="card">
      <h2>OTA Update</h2>
      <form method="POST" action="/update" enctype="multipart/form-data">
        <label for="firmwareFile">Firmware File</label>
        <input id="firmwareFile" type="file" name="firmware" accept=".bin" required />
        <button type="submit">Flash Firmware</button>
      </form>
      <p class="muted mini">Use the OTA `.bin` file only. The board reboots automatically on success.</p>
    </div>
    <div class="card">
      <h2>Backup / Restore</h2>
      <div class="inline">
        <button onclick="downloadBackup()">Download Backup JSON</button>
      </div>
      <label for="restoreFile">Restore Backup File</label>
      <input type="file" id="restoreFile" accept="application/json" />
      <button class="secondary" onclick="uploadRestore()">Restore Backup</button>
      <p class="muted mini">Backup contains firmware metadata, schema version, configuration, saved tags and the recent event ring buffer.</p>
    </div>
  </div>
</div>

<script>
async function api(url, method='GET', body=null) {
  const opts = { method, headers: {} };
  if (body !== null) {
    opts.headers['Content-Type'] = 'application/json';
    opts.body = JSON.stringify(body);
  }
  const r = await fetch(url, opts);
  const ct = r.headers.get('content-type') || '';
  const text = await r.text();
  if (!r.ok) throw new Error(text || r.statusText);
  if (ct.includes('application/json')) return JSON.parse(text);
  return text;
}

function byId(id) { return document.getElementById(id); }

function showBanner(msg, kind='ok') {
  const el = byId('banner');
  el.textContent = msg;
  el.className = `banner ${kind}`;
  clearTimeout(showBanner._t);
  showBanner._t = setTimeout(() => { el.className = 'banner'; el.textContent = ''; }, 5000);
}

function setPill(id, label, good) {
  const el = byId(id);
  el.textContent = label;
  el.style.background = good ? '#e1f6e7' : '#f8e2e2';
}

async function loadStatus() {
  const s = await api('/api/status');
  byId('status').textContent = JSON.stringify(s, null, 2);
  setPill('wifiPill', s.wifi_connected ? `wifi ${s.wifi_ip}` : 'wifi offline', !!s.wifi_connected);
  setPill('mqttPill', s.mqtt_connected ? 'mqtt connected' : `mqtt state ${s.mqtt_state_code}`, !!s.mqtt_connected);
  setPill('readerPill', `reader ${s.reader_state}`, s.reader_state === 'ready');
  setPill('doorPill', `door ${s.output_status}`, !s.output_lockout);
}

async function loadDiagnostics() {
  const d = await api('/api/diagnostics');
  byId('diag').textContent = JSON.stringify(d, null, 2);
}

async function loadLogs() {
  const data = await api('/api/logs');
  byId('logs').textContent = data.logs.join('\n');
}

async function loadEvents() {
  const data = await api('/api/events');
  byId('events').textContent = JSON.stringify(data, null, 2);
}

async function loadConfig() {
  const c = await api('/api/config');
  Object.keys(c).forEach(k => {
    const e = byId(k);
    if (!e) return;
    if (e.type === 'checkbox') e.checked = !!c[k];
    else e.value = c[k] ?? '';
  });
}

async function saveConfig() {
  const body = {
    deviceName: byId('deviceName').value,
    wifiSsid: byId('wifiSsid').value,
    wifiPassword: byId('wifiPassword').value,
    apPassword: byId('apPassword').value,
    mqttHost: byId('mqttHost').value,
    mqttPort: Number(byId('mqttPort').value || 1883),
    mqttUser: byId('mqttUser').value,
    mqttPassword: byId('mqttPassword').value,
    mqttTopicPrefix: byId('mqttTopicPrefix').value,
    mqttDiscovery: byId('mqttDiscovery').checked,
    tzOffsetMin: Number(byId('tzOffsetMin').value || 0),
    scheduleEnabled: byId('scheduleEnabled').checked,
    scheduleDays: byId('scheduleDays').value,
    scheduleStart: byId('scheduleStart').value,
    scheduleEnd: byId('scheduleEnd').value,
    cooldownMs: Number(byId('cooldownMs').value || 0),
    outputEnabled: byId('outputEnabled').checked,
    outputPin: Number(byId('outputPin').value || 0),
    outputActiveHigh: byId('outputActiveHigh').checked,
    outputPulseMs: Number(byId('outputPulseMs').value || 0),
    outputMaxPulseMs: Number(byId('outputMaxPulseMs').value || 0),
    outputIdleMode: byId('outputIdleMode').value,
    outputLockoutLimit: Number(byId('outputLockoutLimit').value || 0),
    outputLockoutWindowMs: Number(byId('outputLockoutWindowMs').value || 0),
    outputLockoutMs: Number(byId('outputLockoutMs').value || 0),
    pinSck: Number(byId('pinSck').value || 0),
    pinMiso: Number(byId('pinMiso').value || 0),
    pinMosi: Number(byId('pinMosi').value || 0),
    pinSs: Number(byId('pinSs').value || 0),
    pinRst: Number(byId('pinRst').value || -1),
    readerPollMs: Number(byId('readerPollMs').value || 0),
    readerRemoveMs: Number(byId('readerRemoveMs').value || 0),
    readerSuppressMs: Number(byId('readerSuppressMs').value || 0),
  };
  const res = await api('/api/config', 'POST', body);
  showBanner(res.message || 'Configuration saved');
  await refreshAll();
}

async function setLearning(enabled, nextOnly) {
  await api('/api/tags/learn', 'POST', { enabled, next_only: nextOnly, name: byId('learnName').value });
  showBanner('Learning state updated');
  await loadStatus();
}

async function learnNext() { await setLearning(true, true); }

async function renameTag(uid) {
  const name = prompt('New tag name:');
  if (name === null) return;
  await api('/api/tags/action', 'POST', { uid, action: 'rename', name });
  await loadTags();
}

async function changeTagAction(uid, value) {
  await api('/api/tags/action', 'POST', { uid, action: 'set_action', tag_action: value });
  showBanner(`Tag action set to ${value}`);
  await loadTags();
}

async function tagState(uid, action) {
  await api('/api/tags/action', 'POST', { uid, action });
  await loadTags();
}

async function loadTags() {
  const data = await api('/api/tags');
  const rows = [
    '<tr><th>UID</th><th>Name</th><th>State</th><th>Tag Action</th><th>Stats</th><th>Actions</th></tr>'
  ];
  for (const t of data.tags) {
    rows.push(`<tr>
      <td>${t.uid}</td>
      <td>${t.name || ''}</td>
      <td>${t.state}</td>
      <td>
        <select onchange="changeTagAction('${t.uid}', this.value)">
          <option value="none" ${t.action === 'none' ? 'selected' : ''}>none</option>
          <option value="pulse" ${t.action === 'pulse' ? 'selected' : ''}>pulse</option>
          <option value="toggle" ${t.action === 'toggle' ? 'selected' : ''}>toggle</option>
        </select>
      </td>
      <td>scans=${t.scans || 0}<br/>last=${t.last_seen_ms || 0} ms</td>
      <td>
        <button onclick="tagState('${t.uid}','allow')">Allow</button>
        <button class="secondary" onclick="tagState('${t.uid}','block')">Block</button>
        <button class="secondary" onclick="tagState('${t.uid}','disable')">Disable</button>
        <button class="secondary" onclick="renameTag('${t.uid}')">Rename</button>
        <button class="danger" onclick="tagState('${t.uid}','delete')">Delete</button>
      </td>
    </tr>`);
  }
  byId('tagsTbl').innerHTML = rows.join('');
}

async function rebootNow() {
  if (!confirm('Reboot device now?')) return;
  await api('/api/reboot', 'POST', {});
  showBanner('Reboot requested');
}

async function pulseOutput() {
  const res = await api('/api/output', 'POST', { action: 'pulse' });
  showBanner(res.message || 'Pulse triggered');
  await loadStatus();
}

async function outputOff() {
  const res = await api('/api/output', 'POST', { action: 'off' });
  showBanner(res.message || 'Output set idle');
  await loadStatus();
}

async function downloadBackup() {
  const data = await api('/api/backup');
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `rfid-backup-${Date.now()}.json`;
  a.click();
}

async function uploadRestore() {
  const file = byId('restoreFile').files[0];
  if (!file) return alert('Select a backup JSON file first.');
  const parsed = JSON.parse(await file.text());
  const res = await api('/api/restore', 'POST', parsed);
  showBanner(res.message || 'Restore complete, rebooting');
}

async function refreshAll() {
  await Promise.all([loadStatus(), loadDiagnostics(), loadLogs(), loadConfig(), loadTags(), loadEvents()]);
}

setInterval(loadStatus, 3000);
setInterval(loadLogs, 4000);
setInterval(loadEvents, 5000);
setInterval(loadDiagnostics, 8000);
refreshAll().catch(err => showBanner(err.message || String(err), 'err'));
</script>
</body>
</html>
)html";

static void setupWebServer() {
  g_server.on("/", HTTP_GET, []() {
    g_server.send_P(200, "text/html", INDEX_HTML);
  });

  g_server.on("/generate_204", HTTP_GET, []() {
    g_server.sendHeader("Location", "/", true);
    g_server.send(302, "text/plain", "");
  });
  g_server.on("/hotspot-detect.html", HTTP_GET, []() {
    g_server.sendHeader("Location", "/", true);
    g_server.send(302, "text/plain", "");
  });
  g_server.on("/fwlink", HTTP_GET, []() {
    g_server.sendHeader("Location", "/", true);
    g_server.send(302, "text/plain", "");
  });

  g_server.on("/api/status", HTTP_GET, []() {
    DynamicJsonDocument doc(1536);
    doc["device"] = g_cfg.deviceName;
    doc["chip_id"] = g_deviceId;
    doc["fw_version"] = FW_VERSION;
    doc["build_stamp"] = BUILD_STAMP;
    doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
    doc["wifi_ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("");
    doc["ap_active"] = g_apActive;
    doc["ap_ip"] = WiFi.softAPIP().toString();
    doc["mqtt_connected"] = g_mqtt.connected();
    doc["mqtt_state_code"] = g_mqtt.state();
    doc["mdns"] = g_mdnsStarted ? (g_cfg.deviceName + ".local") : String("");
    doc["learning"] = g_learningMode;
    doc["learning_next_only"] = g_learningNextOnly;
    doc["learning_name"] = g_learningName;
    doc["tags_count"] = (int)g_tags.size();
    doc["last_uid"] = g_lastUid;
    doc["last_name"] = g_lastName;
    doc["last_event"] = g_lastEvent;
    doc["reader_state"] = g_reader ? readerStateToString(g_reader->state()) : "none";
    doc["reader_status"] = g_reader ? g_reader->statusText() : String("missing");
    doc["output_status"] = g_door.statusText();
    doc["output_lockout"] = g_door.isLockedOut();
    doc["output_state"] = g_door.currentLogicalState();
    doc["output_pulse_remaining_ms"] = g_door.pulseRemainingMs();
    doc["free_heap"] = ESP.getFreeHeap();

    String payload;
    serializeJson(doc, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/diagnostics", HTTP_GET, []() {
    DynamicJsonDocument doc(4096);
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* boot = esp_ota_get_boot_partition();
    doc["device"] = g_cfg.deviceName;
    doc["chip_id"] = g_deviceId;
    doc["fw_version"] = FW_VERSION;
    doc["build_stamp"] = BUILD_STAMP;
    doc["config_schema_current"] = CONFIG_SCHEMA_VERSION;
    doc["config_schema_loaded"] = g_loadedConfigSchema;
    doc["reset_reason"] = g_resetReason;
    doc["uptime_s"] = millis() / 1000;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["fs_available"] = g_fsAvailable;
    doc["wifi_status_code"] = (int)WiFi.status();
    doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
    doc["wifi_ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("");
    doc["wifi_rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
    doc["ap_active"] = g_apActive;
    doc["ap_ip"] = WiFi.softAPIP().toString();
    doc["mqtt_connected"] = g_mqtt.connected();
    doc["mqtt_state_code"] = g_mqtt.state();
    doc["mqtt_base_topic"] = g_baseTopic;
    doc["reader_name"] = g_reader ? g_reader->readerName() : String("none");
    doc["reader_state"] = g_reader ? readerStateToString(g_reader->state()) : "none";
    doc["reader_status"] = g_reader ? g_reader->statusText() : String("missing");
    doc["reader_uid"] = g_reader ? g_reader->currentUid() : String("");
    doc["reader_uid_age_ms"] = g_reader ? g_reader->currentUidAgeMs() : 0;
    doc["reader_error_code"] = g_reader ? g_reader->lastErrorCode() : -1;
    doc["door_status"] = g_door.statusText();
    doc["door_enabled"] = g_door.enabled();
    doc["door_active"] = g_door.isActive();
    doc["door_logical_state"] = g_door.currentLogicalState();
    doc["door_lockout"] = g_door.isLockedOut();
    doc["door_pulse_remaining_ms"] = g_door.pulseRemainingMs();
    doc["door_lockout_remaining_ms"] = g_door.lockoutRemainingMs();
    doc["door_activation_count"] = g_door.activationCountInWindow();
    doc["recent_events_count"] = (int)g_recentEvents.size();
    doc["running_partition"] = partitionSummary(running);
    doc["boot_partition"] = partitionSummary(boot);
    doc["running_partition_state"] = otaImageStateToString(running);
    doc["boot_partition_state"] = otaImageStateToString(boot);

    String payload;
    serializeJson(doc, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/config", HTTP_GET, []() {
    DynamicJsonDocument doc(2048);
    doc["deviceName"] = g_cfg.deviceName;
    doc["wifiSsid"] = g_cfg.wifiSsid;
    doc["wifiPassword"] = g_cfg.wifiPassword;
    doc["apPassword"] = g_cfg.apPassword;
    doc["mqttHost"] = g_cfg.mqttHost;
    doc["mqttPort"] = g_cfg.mqttPort;
    doc["mqttUser"] = g_cfg.mqttUser;
    doc["mqttPassword"] = g_cfg.mqttPassword;
    doc["mqttTopicPrefix"] = g_cfg.mqttTopicPrefix;
    doc["mqttDiscovery"] = g_cfg.mqttDiscovery;
    doc["tzOffsetMin"] = g_cfg.tzOffsetMin;
    doc["scheduleEnabled"] = g_cfg.scheduleEnabled;
    doc["scheduleDays"] = g_cfg.scheduleDays;
    doc["scheduleStart"] = g_cfg.scheduleStart;
    doc["scheduleEnd"] = g_cfg.scheduleEnd;
    doc["cooldownMs"] = g_cfg.cooldownMs;
    doc["outputEnabled"] = g_cfg.outputEnabled;
    doc["outputPin"] = g_cfg.outputPin;
    doc["outputActiveHigh"] = g_cfg.outputActiveHigh;
    doc["outputPulseMs"] = g_cfg.outputPulseMs;
    doc["outputMaxPulseMs"] = g_cfg.outputMaxPulseMs;
    doc["outputIdleMode"] = g_cfg.outputIdleMode;
    doc["outputLockoutLimit"] = g_cfg.outputLockoutLimit;
    doc["outputLockoutWindowMs"] = g_cfg.outputLockoutWindowMs;
    doc["outputLockoutMs"] = g_cfg.outputLockoutMs;
    doc["pinSck"] = g_cfg.pinSck;
    doc["pinMiso"] = g_cfg.pinMiso;
    doc["pinMosi"] = g_cfg.pinMosi;
    doc["pinSs"] = g_cfg.pinSs;
    doc["pinRst"] = g_cfg.pinRst;
    doc["readerPollMs"] = g_cfg.readerPollMs;
    doc["readerRemoveMs"] = g_cfg.readerRemoveMs;
    doc["readerSuppressMs"] = g_cfg.readerSuppressMs;

    String payload;
    serializeJson(doc, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/config", HTTP_POST, []() {
    DynamicJsonDocument doc(2048);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }

    bool wifiChanged = false;
    bool mqttChanged = false;
    bool topicChanged = false;
    bool readerChanged = false;
    bool outputChanged = false;
    bool deviceNameChanged = false;
    bool apChanged = false;

    auto updateIntField = [&](int& dst, const char* key, bool& changedFlag) {
      if (!doc[key].isNull()) {
        int value = doc[key].as<int>();
        if (dst != value) {
          dst = value;
          changedFlag = true;
        }
      }
    };
    auto updateUIntField = [&](uint32_t& dst, const char* key, bool& changedFlag) {
      if (!doc[key].isNull()) {
        uint32_t value = doc[key].as<uint32_t>();
        if (dst != value) {
          dst = value;
          changedFlag = true;
        }
      }
    };

    if (!doc["deviceName"].isNull()) {
      String value = doc["deviceName"].as<String>();
      if (value.isEmpty()) value = "rfid-esp32";
      if (g_cfg.deviceName != value) {
        g_cfg.deviceName = value;
        deviceNameChanged = true;
        topicChanged = true;
      }
    }
    if (!doc["wifiSsid"].isNull()) {
      String value = doc["wifiSsid"].as<String>();
      if (g_cfg.wifiSsid != value) {
        g_cfg.wifiSsid = value;
        wifiChanged = true;
      }
    }
    if (!doc["wifiPassword"].isNull()) {
      String value = doc["wifiPassword"].as<String>();
      if (g_cfg.wifiPassword != value) {
        g_cfg.wifiPassword = value;
        wifiChanged = true;
      }
    }
    if (!doc["apPassword"].isNull()) {
      String value = doc["apPassword"].as<String>();
      if (!value.isEmpty() && value.length() < 8) {
        sendJson(400, "{\"error\":\"apPassword must be empty or >=8 chars\"}");
        return;
      }
      if (g_cfg.apPassword != value) {
        g_cfg.apPassword = value;
        apChanged = true;
      }
    }
    if (!doc["mqttHost"].isNull()) {
      String value = doc["mqttHost"].as<String>();
      if (g_cfg.mqttHost != value) {
        g_cfg.mqttHost = value;
        mqttChanged = true;
      }
    }
    if (!doc["mqttPort"].isNull()) {
      uint16_t value = doc["mqttPort"].as<uint16_t>();
      if (g_cfg.mqttPort != value) {
        g_cfg.mqttPort = value;
        mqttChanged = true;
      }
    }
    if (!doc["mqttUser"].isNull()) {
      String value = doc["mqttUser"].as<String>();
      if (g_cfg.mqttUser != value) {
        g_cfg.mqttUser = value;
        mqttChanged = true;
      }
    }
    if (!doc["mqttPassword"].isNull()) {
      String value = doc["mqttPassword"].as<String>();
      if (g_cfg.mqttPassword != value) {
        g_cfg.mqttPassword = value;
        mqttChanged = true;
      }
    }
    if (!doc["mqttTopicPrefix"].isNull()) {
      String value = doc["mqttTopicPrefix"].as<String>();
      if (g_cfg.mqttTopicPrefix != value) {
        g_cfg.mqttTopicPrefix = value;
        topicChanged = true;
      }
    }
    if (!doc["mqttDiscovery"].isNull()) {
      bool value = doc["mqttDiscovery"].as<bool>();
      if (g_cfg.mqttDiscovery != value) {
        g_cfg.mqttDiscovery = value;
        mqttChanged = true;
      }
    }
    if (!doc["tzOffsetMin"].isNull()) g_cfg.tzOffsetMin = doc["tzOffsetMin"].as<int>();
    if (!doc["scheduleEnabled"].isNull()) g_cfg.scheduleEnabled = doc["scheduleEnabled"].as<bool>();
    if (!doc["scheduleDays"].isNull()) g_cfg.scheduleDays = doc["scheduleDays"].as<String>();
    if (!doc["scheduleStart"].isNull()) g_cfg.scheduleStart = doc["scheduleStart"].as<String>();
    if (!doc["scheduleEnd"].isNull()) g_cfg.scheduleEnd = doc["scheduleEnd"].as<String>();
    if (!doc["cooldownMs"].isNull()) g_cfg.cooldownMs = doc["cooldownMs"].as<uint32_t>();
    if (!doc["outputEnabled"].isNull()) {
      bool value = doc["outputEnabled"].as<bool>();
      if (g_cfg.outputEnabled != value) {
        g_cfg.outputEnabled = value;
        outputChanged = true;
      }
    }
    updateIntField(g_cfg.outputPin, "outputPin", outputChanged);
    if (!doc["outputActiveHigh"].isNull()) {
      bool value = doc["outputActiveHigh"].as<bool>();
      if (g_cfg.outputActiveHigh != value) {
        g_cfg.outputActiveHigh = value;
        outputChanged = true;
      }
    }
    updateUIntField(g_cfg.outputPulseMs, "outputPulseMs", outputChanged);
    updateUIntField(g_cfg.outputMaxPulseMs, "outputMaxPulseMs", outputChanged);
    if (!doc["outputLockoutLimit"].isNull()) {
      uint8_t value = doc["outputLockoutLimit"].as<uint8_t>();
      if (g_cfg.outputLockoutLimit != value) {
        g_cfg.outputLockoutLimit = value;
        outputChanged = true;
      }
    }
    updateUIntField(g_cfg.outputLockoutWindowMs, "outputLockoutWindowMs", outputChanged);
    updateUIntField(g_cfg.outputLockoutMs, "outputLockoutMs", outputChanged);
    if (!doc["outputIdleMode"].isNull()) {
      String value = doc["outputIdleMode"].as<String>();
      if (!isValidIdleMode(value)) {
        sendJson(400, "{\"error\":\"invalid outputIdleMode\"}");
        return;
      }
      if (g_cfg.outputIdleMode != value) {
        g_cfg.outputIdleMode = value;
        outputChanged = true;
      }
    }
    updateIntField(g_cfg.pinSck, "pinSck", readerChanged);
    updateIntField(g_cfg.pinMiso, "pinMiso", readerChanged);
    updateIntField(g_cfg.pinMosi, "pinMosi", readerChanged);
    updateIntField(g_cfg.pinSs, "pinSs", readerChanged);
    updateIntField(g_cfg.pinRst, "pinRst", readerChanged);
    updateUIntField(g_cfg.readerPollMs, "readerPollMs", readerChanged);
    updateUIntField(g_cfg.readerRemoveMs, "readerRemoveMs", readerChanged);
    updateUIntField(g_cfg.readerSuppressMs, "readerSuppressMs", readerChanged);

    if (g_cfg.outputPulseMs == 0) g_cfg.outputPulseMs = 100;
    if (g_cfg.outputMaxPulseMs == 0) g_cfg.outputMaxPulseMs = g_cfg.outputPulseMs;
    if (g_cfg.outputMaxPulseMs < g_cfg.outputPulseMs) g_cfg.outputMaxPulseMs = g_cfg.outputPulseMs;
    if (g_cfg.readerPollMs < 10) g_cfg.readerPollMs = 10;
    if (g_cfg.readerRemoveMs < g_cfg.readerPollMs) g_cfg.readerRemoveMs = g_cfg.readerPollMs * 3;

    saveConfig();
    applyRuntimeConfigChanges(wifiChanged, mqttChanged, topicChanged, readerChanged, outputChanged, deviceNameChanged, apChanged);

    DynamicJsonDocument resp(384);
    resp["ok"] = true;
    resp["message"] = "Configuration saved and applied";
    resp["wifi_reconnect"] = wifiChanged;
    resp["mqtt_reload"] = mqttChanged || topicChanged;
    resp["reader_restart"] = readerChanged;
    resp["output_reconfigured"] = outputChanged;
    String payload;
    serializeJson(resp, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/tags", HTTP_GET, []() {
    sendJson(200, tagsToJson());
  });

  g_server.on("/api/tags/learn", HTTP_POST, []() {
    DynamicJsonDocument doc(512);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }
    g_learningMode = doc["enabled"] | true;
    g_learningNextOnly = doc["next_only"] | false;
    g_learningName = doc["name"] | "";
    publishLearningState();
    publishState();
    sendJson(200, "{\"ok\":true,\"message\":\"learning updated\"}");
  });

  g_server.on("/api/tags/action", HTTP_POST, []() {
    DynamicJsonDocument doc(1024);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }

    String uid = doc["uid"] | "";
    uid.toUpperCase();
    String action = doc["action"] | "";
    if (uid.isEmpty()) {
      sendJson(400, "{\"error\":\"uid required\"}");
      return;
    }

    if (action == "delete") {
      bool removed = false;
      for (size_t i = 0; i < g_tags.size(); ++i) {
        if (g_tags[i].uid == uid) {
          g_tags.erase(g_tags.begin() + i);
          removed = true;
          break;
        }
      }
      if (removed) saveTags();
      publishState();
      sendJson(200, "{\"ok\":true}");
      return;
    }

    TagInfo* tag = findTag(uid);
    if (tag == nullptr) {
      sendJson(404, "{\"error\":\"tag not found\"}");
      return;
    }

    if (action == "allow") tag->state = TagState::Allowed;
    else if (action == "block") tag->state = TagState::Blocked;
    else if (action == "disable") tag->state = TagState::Disabled;
    else if (action == "set_action") {
      String tagAction = doc["tag_action"] | "none";
      if (!isValidTagAction(tagAction)) {
        sendJson(400, "{\"error\":\"invalid tag_action\"}");
        return;
      }
      tag->action = tagAction;
    } else if (action == "rename") {
      tag->name = doc["name"] | tag->name;
    } else {
      sendJson(400, "{\"error\":\"unknown action\"}");
      return;
    }

    saveTags();
    publishState();
    sendJson(200, "{\"ok\":true}");
  });

  g_server.on("/api/events", HTTP_GET, []() {
    sendJson(200, eventsToJson());
  });

  g_server.on("/api/output", HTTP_POST, []() {
    DynamicJsonDocument doc(256);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }
    String action = doc["action"] | "";
    bool ok = false;
    if (action == "pulse") ok = requestOutputPulse("web");
    else if (action == "on") ok = setOutputManual(true, "web");
    else if (action == "off") ok = setOutputManual(false, "web");
    else {
      sendJson(400, "{\"error\":\"unknown action\"}");
      return;
    }
    DynamicJsonDocument resp(256);
    resp["ok"] = ok;
    resp["message"] = ok ? "output updated" : "output rejected";
    resp["status"] = g_door.statusText();
    String payload;
    serializeJson(resp, payload);
    sendJson(ok ? 200 : 409, payload);
  });

  g_server.on("/api/mqtt/cmd", HTTP_POST, []() {
    DynamicJsonDocument doc(1024);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }
    handleCommandJson(doc.as<JsonObject>());
    sendJson(200, "{\"ok\":true}");
  });

  g_server.on("/api/backup", HTTP_GET, []() {
    DynamicJsonDocument doc(24576);
    JsonObject meta = doc.createNestedObject("meta");
    meta["format"] = "rfid-esp32-backup";
    meta["fw_version"] = FW_VERSION;
    meta["build_stamp"] = BUILD_STAMP;
    meta["config_schema_version"] = CONFIG_SCHEMA_VERSION;
    meta["device_id"] = g_deviceId;
    meta["created_at"] = currentIsoTime();
    meta["reader_model"] = g_reader ? g_reader->readerName() : String("PN532 SPI");

    JsonObject cfg = doc.createNestedObject("config");
    cfg["deviceName"] = g_cfg.deviceName;
    cfg["wifiSsid"] = g_cfg.wifiSsid;
    cfg["wifiPassword"] = g_cfg.wifiPassword;
    cfg["apPassword"] = g_cfg.apPassword;
    cfg["mqttHost"] = g_cfg.mqttHost;
    cfg["mqttPort"] = g_cfg.mqttPort;
    cfg["mqttUser"] = g_cfg.mqttUser;
    cfg["mqttPassword"] = g_cfg.mqttPassword;
    cfg["mqttTopicPrefix"] = g_cfg.mqttTopicPrefix;
    cfg["mqttDiscovery"] = g_cfg.mqttDiscovery;
    cfg["tzOffsetMin"] = g_cfg.tzOffsetMin;
    cfg["scheduleEnabled"] = g_cfg.scheduleEnabled;
    cfg["scheduleDays"] = g_cfg.scheduleDays;
    cfg["scheduleStart"] = g_cfg.scheduleStart;
    cfg["scheduleEnd"] = g_cfg.scheduleEnd;
    cfg["cooldownMs"] = g_cfg.cooldownMs;
    cfg["outputEnabled"] = g_cfg.outputEnabled;
    cfg["outputPin"] = g_cfg.outputPin;
    cfg["outputActiveHigh"] = g_cfg.outputActiveHigh;
    cfg["outputPulseMs"] = g_cfg.outputPulseMs;
    cfg["outputMaxPulseMs"] = g_cfg.outputMaxPulseMs;
    cfg["outputIdleMode"] = g_cfg.outputIdleMode;
    cfg["outputLockoutLimit"] = g_cfg.outputLockoutLimit;
    cfg["outputLockoutWindowMs"] = g_cfg.outputLockoutWindowMs;
    cfg["outputLockoutMs"] = g_cfg.outputLockoutMs;
    cfg["pinSck"] = g_cfg.pinSck;
    cfg["pinMiso"] = g_cfg.pinMiso;
    cfg["pinMosi"] = g_cfg.pinMosi;
    cfg["pinSs"] = g_cfg.pinSs;
    cfg["pinRst"] = g_cfg.pinRst;
    cfg["readerPollMs"] = g_cfg.readerPollMs;
    cfg["readerRemoveMs"] = g_cfg.readerRemoveMs;
    cfg["readerSuppressMs"] = g_cfg.readerSuppressMs;

    JsonArray tags = doc.createNestedArray("tags");
    for (const auto& tag : g_tags) {
      JsonObject o = tags.createNestedObject();
      o["uid"] = tag.uid;
      o["name"] = tag.name;
      o["state"] = stateToString(tag.state);
      o["action"] = tag.action;
    }

    JsonArray events = doc.createNestedArray("recent_events");
    for (const auto& ev : g_recentEvents) {
      JsonObject o = events.createNestedObject();
      o["timestamp_ms"] = ev.timestampMs;
      o["uid"] = ev.uid;
      o["name"] = ev.name;
      o["event"] = ev.event;
      o["published"] = ev.published;
    }

    String payload;
    serializeJson(doc, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/restore", HTTP_POST, []() {
    DynamicJsonDocument doc(24576);
    auto err = deserializeJson(doc, readBody());
    if (err) {
      sendJson(400, "{\"error\":\"invalid json\"}");
      return;
    }
    String validationError;
    if (!validateBackup(doc, validationError)) {
      DynamicJsonDocument resp(256);
      resp["error"] = validationError;
      String payload;
      serializeJson(resp, payload);
      sendJson(400, payload);
      return;
    }

    JsonObject cfg = doc["config"].as<JsonObject>();
    if (!cfg["deviceName"].isNull()) g_cfg.deviceName = cfg["deviceName"].as<String>();
    if (!cfg["wifiSsid"].isNull()) g_cfg.wifiSsid = cfg["wifiSsid"].as<String>();
    if (!cfg["wifiPassword"].isNull()) g_cfg.wifiPassword = cfg["wifiPassword"].as<String>();
    if (!cfg["apPassword"].isNull()) g_cfg.apPassword = cfg["apPassword"].as<String>();
    if (!cfg["mqttHost"].isNull()) g_cfg.mqttHost = cfg["mqttHost"].as<String>();
    if (!cfg["mqttPort"].isNull()) g_cfg.mqttPort = cfg["mqttPort"].as<uint16_t>();
    if (!cfg["mqttUser"].isNull()) g_cfg.mqttUser = cfg["mqttUser"].as<String>();
    if (!cfg["mqttPassword"].isNull()) g_cfg.mqttPassword = cfg["mqttPassword"].as<String>();
    if (!cfg["mqttTopicPrefix"].isNull()) g_cfg.mqttTopicPrefix = cfg["mqttTopicPrefix"].as<String>();
    if (!cfg["mqttDiscovery"].isNull()) g_cfg.mqttDiscovery = cfg["mqttDiscovery"].as<bool>();
    if (!cfg["tzOffsetMin"].isNull()) g_cfg.tzOffsetMin = cfg["tzOffsetMin"].as<int>();
    if (!cfg["scheduleEnabled"].isNull()) g_cfg.scheduleEnabled = cfg["scheduleEnabled"].as<bool>();
    if (!cfg["scheduleDays"].isNull()) g_cfg.scheduleDays = cfg["scheduleDays"].as<String>();
    if (!cfg["scheduleStart"].isNull()) g_cfg.scheduleStart = cfg["scheduleStart"].as<String>();
    if (!cfg["scheduleEnd"].isNull()) g_cfg.scheduleEnd = cfg["scheduleEnd"].as<String>();
    if (!cfg["cooldownMs"].isNull()) g_cfg.cooldownMs = cfg["cooldownMs"].as<uint32_t>();
    if (!cfg["outputEnabled"].isNull()) g_cfg.outputEnabled = cfg["outputEnabled"].as<bool>();
    if (!cfg["outputPin"].isNull()) g_cfg.outputPin = cfg["outputPin"].as<int>();
    if (!cfg["outputActiveHigh"].isNull()) g_cfg.outputActiveHigh = cfg["outputActiveHigh"].as<bool>();
    if (!cfg["outputPulseMs"].isNull()) g_cfg.outputPulseMs = cfg["outputPulseMs"].as<uint32_t>();
    if (!cfg["outputMaxPulseMs"].isNull()) g_cfg.outputMaxPulseMs = cfg["outputMaxPulseMs"].as<uint32_t>();
    if (!cfg["outputIdleMode"].isNull()) g_cfg.outputIdleMode = cfg["outputIdleMode"].as<String>();
    if (!cfg["outputLockoutLimit"].isNull()) g_cfg.outputLockoutLimit = cfg["outputLockoutLimit"].as<uint8_t>();
    if (!cfg["outputLockoutWindowMs"].isNull()) g_cfg.outputLockoutWindowMs = cfg["outputLockoutWindowMs"].as<uint32_t>();
    if (!cfg["outputLockoutMs"].isNull()) g_cfg.outputLockoutMs = cfg["outputLockoutMs"].as<uint32_t>();
    if (!cfg["pinSck"].isNull()) g_cfg.pinSck = cfg["pinSck"].as<int>();
    if (!cfg["pinMiso"].isNull()) g_cfg.pinMiso = cfg["pinMiso"].as<int>();
    if (!cfg["pinMosi"].isNull()) g_cfg.pinMosi = cfg["pinMosi"].as<int>();
    if (!cfg["pinSs"].isNull()) g_cfg.pinSs = cfg["pinSs"].as<int>();
    if (!cfg["pinRst"].isNull()) g_cfg.pinRst = cfg["pinRst"].as<int>();
    if (!cfg["readerPollMs"].isNull()) g_cfg.readerPollMs = cfg["readerPollMs"].as<uint32_t>();
    if (!cfg["readerRemoveMs"].isNull()) g_cfg.readerRemoveMs = cfg["readerRemoveMs"].as<uint32_t>();
    if (!cfg["readerSuppressMs"].isNull()) g_cfg.readerSuppressMs = cfg["readerSuppressMs"].as<uint32_t>();

    g_tags.clear();
    JsonArray tags = doc["tags"].as<JsonArray>();
    for (JsonVariant v : tags) {
      TagInfo tag;
      tag.uid = v["uid"] | "";
      tag.name = v["name"] | "";
      tag.state = stringToState(v["state"] | "allowed");
      tag.action = v["action"] | "none";
      if (!tag.uid.isEmpty()) g_tags.push_back(tag);
    }

    g_recentEvents.clear();
    JsonArray events = doc["recent_events"].as<JsonArray>();
    for (JsonVariant v : events) {
      RecentEvent ev;
      ev.timestampMs = v["timestamp_ms"] | 0;
      ev.uid = v["uid"] | "";
      ev.name = v["name"] | "";
      ev.event = v["event"] | "";
      ev.published = v["published"] | false;
      if (!ev.uid.isEmpty() || !ev.event.isEmpty()) g_recentEvents.push_back(ev);
    }

    saveConfig();
    saveTags();
    saveRecentEventsNow();
    sendJson(200, "{\"ok\":true,\"message\":\"Restore complete, rebooting\"}");
    delay(500);
    ESP.restart();
  });

  g_server.on("/api/logs", HTTP_GET, []() {
    DynamicJsonDocument doc(12288);
    JsonArray arr = doc.createNestedArray("logs");
    for (const auto& line : g_logBuffer) arr.add(line);
    String payload;
    serializeJson(doc, payload);
    sendJson(200, payload);
  });

  g_server.on("/api/reboot", HTTP_POST, []() {
    sendJson(200, "{\"ok\":true,\"rebooting\":true}");
    addLog("Reboot requested via web API");
    delay(300);
    ESP.restart();
  });

  g_server.on("/update", HTTP_POST,
    []() {
      bool ok = !Update.hasError();
      if (ok) {
        g_server.send(200, "text/plain", "OK");
      } else {
        String err = String("FAIL error=") + String(Update.getError());
        g_server.send(200, "text/plain", err);
        addLog(String("OTA finalize failed, error=") + String(Update.getError()));
      }
      delay(500);
      if (ok) ESP.restart();
    },
    []() {
      HTTPUpload& upload = g_server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        addLog(String("OTA start: ") + upload.filename);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
          addLog(String("OTA begin failed, error=") + String(Update.getError()));
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          addLog(String("OTA write failed, error=") + String(Update.getError()));
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        addLog(String("OTA end, bytes=") + String(upload.totalSize));
        if (!Update.end(true)) {
          addLog(String("OTA end failed, error=") + String(Update.getError()));
          Update.printError(Serial);
        } else {
          addLog("OTA success, rebooting");
        }
      }
    }
  );

  g_server.onNotFound([]() {
    if (g_apActive) {
      g_server.sendHeader("Location", "/", true);
      g_server.send(302, "text/plain", "");
      return;
    }
    g_server.send(404, "application/json", "{\"error\":\"not found\"}");
  });

  g_server.begin();
}

static void handleScan(const String& uid) {
  TagInfo* tag = findTag(uid);

  if (g_learningMode) {
    if (tag == nullptr) {
      TagInfo n;
      n.uid = uid;
      n.name = g_learningName.length() > 0 ? g_learningName : ("Tag-" + uid.substring(uid.length() > 4 ? uid.length() - 4 : 0));
      n.state = TagState::Allowed;
      g_tags.push_back(n);
      tag = &g_tags.back();
      saveTags();
      addLog(String("Learned new tag uid=") + uid + " name=" + tag->name);
    } else if (g_learningName.length() > 0) {
      tag->name = g_learningName;
      saveTags();
      addLog(String("Updated learned tag uid=") + uid + " name=" + tag->name);
    }

    if (g_learningNextOnly) {
      g_learningMode = false;
      g_learningNextOnly = false;
      g_learningName = "";
      publishLearningState();
    }
  }

  String eventType = "unknown";
  if (tag != nullptr) {
    tag->scans++;
    tag->lastSeenMs = millis();
    if (g_cfg.cooldownMs > 0 && (millis() - tag->lastDecisionMs) < g_cfg.cooldownMs) {
      eventType = "cooldown";
    } else if (tag->state == TagState::Allowed) {
      if (scheduleAllowsNow()) {
        eventType = "allowed";
        tag->lastDecisionMs = millis();
        runTagAction(tag);
        blinkStatusLed(1, 50, 20);
      } else {
        eventType = "schedule_blocked";
        blinkStatusLed(2, 40, 30);
      }
    } else if (tag->state == TagState::Blocked) {
      eventType = "blocked";
      blinkStatusLed(3, 40, 30);
    } else {
      eventType = "disabled";
      blinkStatusLed(2, 100, 40);
    }
  } else {
    blinkStatusLed(1, 20, 20);
  }

  g_lastUid = uid;
  g_lastName = tag ? tag->name : "";
  g_lastEvent = eventType;
  queueRecentEvent(uid, g_lastName, eventType);
  addLog(String("RFID scan uid=") + uid + " event=" + eventType + " name=" + g_lastName);

  if (!g_recentEvents.empty()) {
    publishRecentEventAt(g_recentEvents.size() - 1);
  }
  publishState();
}

static void processReader() {
  if (g_reader == nullptr) return;
  g_reader->loop();

  ReaderState readerState = g_reader->state();
  if (readerState != g_prevReaderState) {
    g_prevReaderState = readerState;
    addLog(String("Reader state changed: ") + readerStateToString(readerState) + " status=" + g_reader->statusText());
    publishState();
  }

  TagReadResult result;
  if (g_reader->poll(result)) {
    handleScan(result.uid);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  g_resetReason = resetReasonToString(esp_reset_reason());
  ensureOtaStateValid();

  g_deviceId = getChipId();
  rebuildBaseTopic();

  g_fsAvailable = LittleFS.begin(true);
  if (!g_fsAvailable) addLog("LittleFS mount failed");
  else addLog("LittleFS mounted");

  g_prefs.begin(PREF_NAMESPACE, false);
  loadConfig();
  loadTags();
  loadRecentEvents();
  rebuildBaseTopic();

  configTime(g_cfg.tzOffsetMin * 60, 0, "pool.ntp.org", "time.nist.gov");

  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(g_cfg.deviceName.c_str());
  connectWifiIfNeeded(true);
  ensureAccessPointIfNeeded();

  configureDoorController();
  restartReader();
  setupWebServer();

  addLog(String("Device: ") + g_cfg.deviceName + " (" + g_deviceId + ") FW=" + FW_VERSION + " build=" + BUILD_STAMP);
  addLog(String("Reset reason: ") + g_resetReason);
  addLog("HTTP server started on port 80");
}

void loop() {
  g_server.handleClient();
  if (g_dnsActive) g_dns.processNextRequest();

  if (WiFi.status() != WL_CONNECTED) {
    connectWifiIfNeeded();
    ensureAccessPointIfNeeded();
  }

  wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != g_prevWiFiStatus) {
    g_prevWiFiStatus = wifiStatus;
    if (wifiStatus == WL_CONNECTED) {
      addLog(String("WiFi connected, IP=") + WiFi.localIP().toString());
      restartMdnsIfNeeded();
    } else {
      addLog(String("WiFi status changed: ") + String((int)wifiStatus));
      if (g_mdnsStarted) {
        MDNS.end();
        g_mdnsStarted = false;
      }
    }
    ensureAccessPointIfNeeded();
    publishState();
  }

  connectMqttIfNeeded();
  g_mqtt.loop();

  int mqttState = g_mqtt.state();
  if (mqttState != g_prevMqttState) {
    g_prevMqttState = mqttState;
    addLog(String("MQTT state changed: ") + String(mqttState));
  }

  g_door.loop();
  if (g_door.currentLogicalState() != g_prevDoorLogicalState || g_door.statusText() != g_prevDoorStatus) {
    g_prevDoorLogicalState = g_door.currentLogicalState();
    g_prevDoorStatus = g_door.statusText();
    publishOutputState();
    publishState();
  }

  processReader();

  if (millis() - g_lastStatePublishMs > 30000) {
    g_lastStatePublishMs = millis();
    publishState();
    publishOutputState();
    publishPendingEvents();
  }

  if (g_eventsDirty && millis() - g_lastEventsSaveMs > 2000) {
    saveRecentEventsNow();
  }

  if (millis() - g_lastDiagnosticsLogMs > 60000) {
    g_lastDiagnosticsLogMs = millis();
    addLog(String("Heartbeat heap=") + String(ESP.getFreeHeap()) + " reader=" + (g_reader ? g_reader->statusText() : String("missing")) + " door=" + g_door.statusText());
  }

  delay(5);
}
