#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>

// WiFi credentials
const char* ssid = "Airtel_X25A_E048_5G";
const char* password = "CF6903AD";

// ThingSpeak settings
const char* server = "http://api.thingspeak.com";
String apiKey = "FEPZY6X8U6QLA5O2";

// GPS on UART2
static const int RXPin = 16;   // ESP32 RX  (connect to GPS TX)
static const int TXPin = 17;   // ESP32 TX  (connect to GPS RX)
static const uint32_t GPSBaud = 9600;

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

unsigned long lastSend = 0;
const unsigned long interval = 20000; // 20 sec (ThingSpeak free limit)

void connectWiFi() {
  Serial.println("Trying to connect to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected successfully.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection FAILED.");

    wl_status_t status = WiFi.status();

    if (status == WL_NO_SSID_AVAIL) {
      Serial.println("Reason: SSID not found.");
    } 
    else if (status == WL_CONNECT_FAILED) {
      Serial.println("Reason: Wrong password or authentication failed.");
    } 
    else if (status == WL_DISCONNECTED) {
      Serial.println("Reason: Disconnected from router.");
    } 
    else {
      Serial.print("Reason code: ");
      Serial.println(status);
    }

    Serial.println("Check:");
    Serial.println("- ESP32 supports ONLY 2.4GHz WiFi.");
    Serial.println("- SSID and password are correct.");
    Serial.println("- Router is nearby.");
  }
}

void sendToThingSpeak(double lat, double lon, double speed, int sats) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  HTTPClient http;

  String url = String(server) + "/update?api_key=" + apiKey +
               "&field1=" + String(lat, 6) +
               "&field2=" + String(lon, 6) +
               "&field3=" + String(speed, 2) +
               "&field4=" + String(sats);

  http.begin(url);
  int httpCode = http.GET();
  http.end();

  Serial.print("ThingSpeak Response: ");
  Serial.println(httpCode);
}

void setup() {
  Serial.begin(115200);

  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  connectWiFi();
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastSend > interval) {
    lastSend = millis();

    if (gps.location.isValid()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      double speedKmph = gps.speed.kmph();
      int satellites = gps.satellites.value();

      Serial.println("Sending data to ThingSpeak...");
      sendToThingSpeak(latitude, longitude, speedKmph, satellites);
    } else {
      Serial.println("Waiting for GPS fix...");
    }
  }
}