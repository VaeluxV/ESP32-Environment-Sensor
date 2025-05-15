#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

#include <secrets.h> // Include passwords, SSID and other info from secrets.h

// === Device Information ===
const char* sensor_id = "env-001";
const char* device_type = "Environmental-tracker";
const char* firmware_version = "v2025.05.08a";
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

// === Setup Wi-Fi connection ===
void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected. IP: " + WiFi.localIP().toString());
}

// === MQTT (re-)connection function ===
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5s");
      delay(5000);
    }
  }
}

// === Read sensors and publish data ===
void publish_sensor_data() {
  float temperature = NAN, humidity = NAN;

  if (bme_status) {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
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
  doc["estimated_altitude"] = 0; // placeholder
  doc["noise_level"] = 0; // placeholder
  doc["firmware_version"] = firmware_version;
  doc["connection_type"] = connection_type;
  doc["uptime_seconds"] = (millis() - startMillis) / 1000;
  doc["battery_level"] = 100; // placeholder
  doc["critical_battery"] = false; // placeholder
  doc["status"] = "active_normal"; // placeholder until made dynamic
  doc["error_code"] = 0;

  char buffer[512];
  size_t len = serializeJson(doc, buffer);
  client.publish("envtracker001/data", buffer, len);
  Serial.println("Published JSON:");
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  startMillis = millis();

  // Initialize BME280
  bme_status = bme.begin(0x76); // Try 0x76 or 0x77 based on your module
  if (!bme_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // Publish every 5 seconds
  if (millis() - lastPublish > publish_interval) {
    lastPublish = millis();
    publish_sensor_data();
  }
}
