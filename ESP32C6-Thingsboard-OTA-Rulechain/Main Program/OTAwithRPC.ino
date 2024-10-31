#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <OTA_Firmware_Update.h>
#include <ThingsBoard.h>
#include <Espressif_Updater.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin configuration
const int ph_pin = A0;
const int relayHeater = 11;  // Replacing Pump A with Heater
const int relayValve = 3;    // Replacing Pump B with Valve
const int ds18b20Pin = 2;    // Pin for DS18B20 sensor

bool heater = false;
bool valve = false;

float PO = 0;
float PH_step;
float nilai_analog_PH;
double tegangan;
float PH4 = 3.01;
float PH7 = 2.52;

// DS18B20 sensor setup
OneWire oneWire(ds18b20Pin);
DallasTemperature sensors(&oneWire);

// Encryption setting
#define ENCRYPTED false

constexpr char CURRENT_FIRMWARE_TITLE[] = "TEST";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.2";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

constexpr char WIFI_SSID[] = "oke";
constexpr char WIFI_PASSWORD[] = "12345678910";
constexpr char TOKEN[] = "AAMJqLH8HKeiOII6NW8o";
constexpr char THINGSBOARD_SERVER[] = "mqtt.thingsboard.cloud";

#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

constexpr uint16_t MAX_MESSAGE_SIZE = 512U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

#if ENCRYPTED
WiFiClientSecure espClient;
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
[Insert SSL Certificate Here]
-----END CERTIFICATE-----)";
#else
WiFiClient espClient;
#endif

Arduino_MQTT_Client mqttClient(espClient);
OTA_Firmware_Update<> ota;
const std::array<IAPI_Implementation*, 1U> apis = {&ota};
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);
Espressif_Updater<> updater;

bool currentFWSent = false;
bool updateRequestSent = false;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
#if ENCRYPTED
  espClient.setCACert(ROOT_CERT);
#endif
}

bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return WiFi.status() == WL_CONNECTED;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String payloadStr;
  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  Serial.println("Payload: " + payloadStr);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payloadStr);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("method") && String(doc["method"]) == "setHeater") {
    bool heaterStatus = doc["params"]["state"];
    digitalWrite(relayHeater, heaterStatus ? LOW : HIGH);
    Serial.println(heaterStatus ? "Heater: ON" : "Heater: OFF");
    heater = heaterStatus;
  }
  
  if (doc.containsKey("method") && String(doc["method"]) == "setValve") {
    bool valveStatus = doc["params"]["state"];
    digitalWrite(relayValve, valveStatus ? LOW : HIGH);
    Serial.println(valveStatus ? "Valve: ON" : "Valve: OFF");
    valve = valveStatus;
  }
}

void on_mqtt_connected() {
  Serial.println("Connected to MQTT broker");
  mqttClient.subscribe("v1/devices/me/rpc/request/+");
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  mqttClient.set_server(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  mqttClient.set_data_callback(mqttCallback);
  mqttClient.set_connect_callback(on_mqtt_connected);
  
  while (!mqttClient.connected()) {
    Serial.println("Connecting to ThingsBoard...");
    if (mqttClient.connect("Arduino_Client", TOKEN, NULL)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.println("Failed to connect, retrying in 5 seconds");
      delay(5000);
    }
  }
  
  pinMode(relayHeater, OUTPUT);
  pinMode(relayValve, OUTPUT);
  digitalWrite(relayHeater, HIGH);
  digitalWrite(relayValve, HIGH);

  sensors.begin();  // Start the DS18B20 sensor
}

void loop() {
  delay(1000);

  if (!reconnect()) return;

  if (!mqttClient.connected()) {
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!mqttClient.connect("ESP32", TOKEN, "")) {
      Serial.println("Failed to connect");
      return;
    }
  }

  mqttClient.loop();

  if (tb.connected()) {
    // pH Sensor
    nilai_analog_PH = analogRead(ph_pin);
    tegangan = 3.3 * nilai_analog_PH / 4095.0;
    PH_step = (PH4 - PH7) / 3;
    PO = 7 + ((PH7 - tegangan) / PH_step);
    Serial.print("Nilai pH : ");
    Serial.println(PO);

    // DS18B20 Sensor
    sensors.requestTemperatures();
    float temperature = sensors.getTempCByIndex(0);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    sendTelemetry(PO, temperature);
  }
}

void sendTelemetry(float phValue, float temperature) {
  if (tb.connected()) {
    String heater_status = heater ? "ON" : "OFF";
    String valve_status = valve ? "ON" : "OFF";
    tb.sendTelemetryData("HEATER", heater_status);
    tb.sendTelemetryData("VALVE", valve_status);
    tb.sendTelemetryData("pH", phValue);
    tb.sendTelemetryData("Temperature", temperature);
  }
}
