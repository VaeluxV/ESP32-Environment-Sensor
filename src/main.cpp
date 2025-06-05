#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include <secrets.h> // Include passwords, SSID and other info from secrets.h

// === Device Information ===
const char* sensor_id = "env-001";
const char* device_type = "Environmental-tracker";
const char* firmware_version = "v2025.06.05a";
const char* connection_type = "Wi-Fi";

WiFiClient espClient;
PubSubClient client(espClient);

// === BME280 Sensor ===
Adafruit_BME280 bme;
bool bme_status = false;

// === LDR ===
const int LDR_PIN = 34; // ADC1 channel 6 (works while Wi-Fi is enabled/in use)

// === Timing ===
unsigned long lastPublish = 0;
unsigned long startMillis;
const int publish_interval = 5000; // 5 seconds

// === Connection state ===
int wifi_retry_delay = 10000; // Start at 10s
unsigned long last_wifi_attempt = 0;

int mqtt_retry_count = 0;
const int mqtt_retry_limit = 5;

unsigned long last_reconnect_attempt = 0;
const unsigned long reconnect_cooldown = 30000; // 30s cooldown before retrying full reconnect

bool full_reconnect_required = false;

// === Disconnect both Wi-Fi and MQTT safely ===
void safe_disconnect_all() {
  Serial.println("Disconnecting Wi-Fi and MQTT...");
  client.disconnect();
  WiFi.disconnect(true);
  delay(100);
}

// === Setup Wi-Fi connection (blocking) ===
void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected. IP: " + WiFi.localIP().toString());
}

// === Check Wi-Fi and apply backoff retry ===
void check_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - last_wifi_attempt >= wifi_retry_delay) {
      Serial.println("Wi-Fi disconnected, attempting to reconnect...");
      WiFi.begin(ssid, password);
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        Serial.print(".");
      }
      Serial.println();
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Wi-Fi reconnected. IP: " + WiFi.localIP().toString());
        wifi_retry_delay = 10000; // reset on success
      } else {
        wifi_retry_delay = min(wifi_retry_delay + 5000, 30000); // backoff up to 30s
        Serial.println("Wi-Fi reconnection failed. Increasing delay.");
      }
      last_wifi_attempt = now;
    }
  }
}

// === Check MQTT connection and apply retry limit ===
void check_mqtt() {
  if (!client.connected()) {
    if (mqtt_retry_count >= mqtt_retry_limit) {
      Serial.println("MQTT failed too many times. Will reconnect everything...");
      full_reconnect_required = true;
      mqtt_retry_count = 0;
      last_reconnect_attempt = millis();
      return;
    }

    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      mqtt_retry_count = 0;
    } else {
      mqtt_retry_count++;
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.print(" retry ");
      Serial.print(mqtt_retry_count);
      Serial.print("/");
      Serial.println(mqtt_retry_limit);
      delay(5000);
    }
  }
}

// === Read sensors and publish JSON data ===
void publish_sensor_data() {
  float temperature = NAN, humidity = NAN;

  if (bme_status) {
    temperature = bme.readTemperature() - 3.5; // Adjusted for calibration
    humidity = bme.readHumidity() / 100.0; // Convert to percentage
  }

  int light_level = analogRead(LDR_PIN);

  JsonDocument doc;
  JsonObject location = doc["location"].to<JsonObject>();
  location["lat"] = dummy_lat;
  location["lon"] = dummy_lon;

  doc["light_level"] = light_level;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["sensor_id"] = sensor_id;
  doc["device_type"] = device_type;
  doc["signal_strength"] = WiFi.RSSI();
  doc["firmware_version"] = firmware_version;
  doc["connection_type"] = connection_type;
  doc["uptime_seconds"] = (millis() - startMillis) / 1000;
  doc["critical_battery"] = false;
  doc["status"] = "active_normal";
  doc["error_code"] = 0;

  char buffer[512];
  size_t len = serializeJson(doc, buffer);
  client.publish("envtracker001/data", buffer, len);
  Serial.println("Published JSON:");
  Serial.println(buffer);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(1024);
  startMillis = millis();

  // Fix swapped I2C pins (use default if needed)
  Wire.begin(22, 21); // SDA, SCL

  bme_status = bme.begin(0x76);
  if (!bme_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
}

// === Main loop ===
void loop() {
  if (full_reconnect_required) {
    if (millis() - last_reconnect_attempt >= reconnect_cooldown) {
      safe_disconnect_all();
      setup_wifi();
      client.setServer(mqtt_server, mqtt_port);
      full_reconnect_required = false;
    } else {
      return; // Wait during cooldown
    }
  }

  check_wifi();

  if (WiFi.status() == WL_CONNECTED) {
    check_mqtt();
    client.loop();

    if (millis() - lastPublish > publish_interval) {
      lastPublish = millis();
      publish_sensor_data();
    }
  }
}
