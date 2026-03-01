/*
ESP32 + NEO-6M GPS logger:
- If Serial Monitor is connected -> logs to Serial
- Otherwise -> logs to MQTT broker (battery mode / no USB)

Publishes (when Serial not available):
- gps/<DEVICE_ID>/status   (retained)  "online"/"offline"
- gps/<DEVICE_ID>/nmea                 raw NMEA lines
- gps/<DEVICE_ID>/fix                  parsed JSON (lat/lon/alt/sats/hdop/speed/course/date/time/fix)

Libraries (Arduino Library Manager):
- TinyGPSPlus (Mikal Hart)
- PubSubClient (Nick O'Leary)

Wiring (adjust pins as needed):
NEO6M TX -> ESP32 GPIO22 (RX2)
NEO6M RX -> ESP32 GPIO23 (TX2) optional
NEO6M GND -> ESP32 GND
NEO6M VCC -> ESP32 3V3 (or 5V if your module supports it)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>

// ===================== USER SETTINGS =====================
// WiFi
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// MQTT
const char* MQTT_HOST = "192.168.1.10";   // broker IP / domain
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";               // set if required
const char* MQTT_PASS = "";               // set if required

// Device/Topics
const char* DEVICE_ID = "esp32-gps-01";

// GPS UART2 pins
static const int GPS_RX = 22;   // ESP32 RX2  <- GPS TX
static const int GPS_TX = 23;   // ESP32 TX2  -> GPS RX (optional)
static const uint32_t GPS_BAUD = 9600;

// Publish interval for parsed JSON
const unsigned long FIX_PUBLISH_INTERVAL_MS = 1000;

// =========================================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// Topics
String topicBase;
String topicStatus;
String topicNmea;
String topicFix;

// Raw NMEA line capture
char nmeaLine[160];
size_t nmeaPos = 0;

// Timing
unsigned long lastFixPublishMs = 0;

// ---------- Helpers ----------
bool serialAvailable() {
  // On ESP32, Serial becomes "true" when the USB serial is connected/opened.
  // If you power from battery and no USB cable, this is usually false.
  return (bool)Serial;
}

void logLine(const String& s) {
  // Always try serial if available; do not also MQTT (your requirement: Serial first, else MQTT)
  if (serialAvailable()) {
    Serial.println(s);
  } else {
    // MQTT logging for battery mode
    if (!mqtt.connected()) return;
    mqtt.publish(topicFix.c_str(), s.c_str());
  }
}

void logNmea(const char* line) {
  if (serialAvailable()) {
    Serial.println(line);
  } else {
    if (!mqtt.connected()) return;
    mqtt.publish(topicNmea.c_str(), line);
  }
}

void wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - start > 15000) break; // don't block forever
  }
}

bool mqttEnsureConnected() {
  if (mqtt.connected()) return true;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  // Attempt for up to ~10s
  unsigned long start = millis();
  while (!mqtt.connected() && (millis() - start < 10000)) {
    String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    bool ok = false;
    if (strlen(MQTT_USER) > 0) {
      ok = mqtt.connect(
        clientId.c_str(),
        MQTT_USER, MQTT_PASS,
        topicStatus.c_str(), 1, true, "offline"
      );
    } else {
      ok = mqtt.connect(
        clientId.c_str(),
        topicStatus.c_str(), 1, true, "offline"
      );
    }

    if (ok) {
      mqtt.publish(topicStatus.c_str(), "online", true);
      return true;
    }
    delay(700);
  }
  return mqtt.connected();
}

void mqttLoopIfUsed() {
  // Only bother with WiFi/MQTT if Serial is NOT available
  if (serialAvailable()) return;

  wifiEnsureConnected();
  if (WiFi.status() == WL_CONNECTED) {
    mqttEnsureConnected();
    mqtt.loop();
  }
}

String buildFixJson() {
  String payload = "{";

  payload += "\"fix\":";
  payload += gps.location.isValid() ? "true" : "false";

  payload += ",\"lat\":";
  payload += gps.location.isValid() ? String(gps.location.lat(), 6) : "null";

  payload += ",\"lon\":";
  payload += gps.location.isValid() ? String(gps.location.lng(), 6) : "null";

  payload += ",\"alt_m\":";
  payload += gps.altitude.isValid() ? String(gps.altitude.meters(), 2) : "null";

  payload += ",\"sats\":";
  payload += gps.satellites.isValid() ? String(gps.satellites.value()) : "null";

  payload += ",\"hdop\":";
  payload += gps.hdop.isValid() ? String(gps.hdop.hdop(), 2) : "null";

  payload += ",\"speed_kmph\":";
  payload += gps.speed.isValid() ? String(gps.speed.kmph(), 2) : "null";

  payload += ",\"course_deg\":";
  payload += gps.course.isValid() ? String(gps.course.deg(), 2) : "null";

  payload += ",\"date\":\"";
  if (gps.date.isValid()) {
    char d[11];
    snprintf(d, sizeof(d), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
    payload += d;
  } else {
    payload += "null";
  }
  payload += "\"";

  payload += ",\"time_utc\":\"";
  if (gps.time.isValid()) {
    char t[9];
    snprintf(t, sizeof(t), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    payload += t;
  } else {
    payload += "null";
  }
  payload += "\"";

  payload += "}";

  return payload;
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  topicBase   = String("gps/") + DEVICE_ID;
  topicStatus = topicBase + "/status";
  topicNmea   = topicBase + "/nmea";
  topicFix    = topicBase + "/fix";

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // If Serial isn't connected at boot, prepare WiFi/MQTT path
  mqttLoopIfUsed();
}

void loop() {
  // Maintain MQTT only when Serial is not connected
  mqttLoopIfUsed();

  // Read GPS stream continuously
  while (GPSSerial.available()) {
    char c = (char)GPSSerial.read();
    gps.encode(c);

    // Capture and log raw NMEA lines
    if (c == '\r') continue;

    if (c == '\n') {
      if (nmeaPos > 0) {
        nmeaLine[nmeaPos] = '\0';
        logNmea(nmeaLine);
        nmeaPos = 0;
      }
    } else {
      if (nmeaPos < sizeof(nmeaLine) - 1) {
        nmeaLine[nmeaPos++] = c;
      } else {
        // overflow -> drop line
        nmeaPos = 0;
      }
    }
  }

  // Publish/print parsed fix at a fixed interval
  unsigned long now = millis();
  if (now - lastFixPublishMs >= FIX_PUBLISH_INTERVAL_MS) {
    lastFixPublishMs = now;

    String fixJson = buildFixJson();

    if (serialAvailable()) {
      Serial.println(fixJson);
    } else {
      if (mqtt.connected()) {
        mqtt.publish(topicFix.c_str(), fixJson.c_str());
      }
    }
  }
}