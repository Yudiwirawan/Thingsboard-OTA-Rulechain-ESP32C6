#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <OTA_Firmware_Update.h>
#include <ThingsBoard.h>
#include <Espressif_Updater.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ENCRYPTED false

constexpr char CURRENT_FIRMWARE_TITLE[] = "TEST";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.0";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

constexpr char WIFI_SSID[] = "oke";
constexpr char WIFI_PASSWORD[] = "12345678910";
constexpr char TOKEN[] = "lbZcFLwDSwtsoMkMKwcz";
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

// Pin definitions
#define ONE_WIRE_BUS 23    // DS18B20 data pin
#define PH_SENSOR_PIN 4  // Analog pin for pH sensor
#define HEATER_PIN 21     // Pin for heater control
#define VALVE_PIN 20      // Pin for valve control

// Temperature control parameters
constexpr float TEMP_SETPOINT = 40.0;    // Temperature setpoint in Celsius
constexpr float TEMP_HYSTERESIS = 0.5;   // Temperature hysteresis

// pH control parameters
constexpr float PH_MIN = 6.0;    // Minimum pH threshold
constexpr float PH_MAX = 9.0;    // Maximum pH threshold

// Initialize OneWire and DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to AP");
#if ENCRYPTED
  espClient.setCACert(ROOT_CERT);
#endif
}

bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return true;
}

void update_starting_callback() {}

void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    esp_restart();
  } else {
    Serial.println("Downloading firmware failed");
  }
}

void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}

// Function to read temperature from DS18B20
float get_temperature() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  
  if(tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error reading temperature!");
    return -127.0;
  }
  return tempC;
}

// Function to read pH value
float get_ph() {
  // Read the analog value from pH sensor
  int analogValue = analogRead(PH_SENSOR_PIN);
  
  // Convert analog reading to pH value (calibration needed)
  // This is a simplified conversion - actual conversion depends on your sensor's specifications
  float voltage = analogValue * (3.3 / 4095.0);
  float ph = 3.5 * voltage + 0.0; // Replace with your calibration formula
  
  return ph;
}

// Function to send telemetry to ThingsBoard
void send_telemetry(float temperature, float ph, bool heaterStatus, bool valveStatus) {
  tb.sendTelemetryData("temperature", temperature);
  tb.sendTelemetryData("pH", ph);
  tb.sendTelemetryData("heaterStatus", heaterStatus ? "ON" : "OFF");
  tb.sendTelemetryData("valveStatus", valveStatus ? "OPEN" : "CLOSED");
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  // Initialize sensors and control pins
  sensors.begin();
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PH_SENSOR_PIN, INPUT);
  
  // Initially turn off both heater and valve
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
}

void loop() {
  delay(1000);

  if (!reconnect()) return;

  if (!tb.connected()) {
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  if (!currentFWSent) {
    currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
  }

  if (!updateRequestSent) {
    Serial.println("Firmware Update...");
    const OTA_Update_Callback callback(
      CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater,
      &finished_callback, &progress_callback, &update_starting_callback,
      FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE
    );
    updateRequestSent = ota.Start_Firmware_Update(callback);
  }

  // Read sensor values
  float temperature = get_temperature();
  float ph = get_ph();
  
  Serial.printf("Temperature: %.2f°C\n", temperature);
  Serial.printf("pH: %.2f\n", ph);

  // Control logic
  bool heaterStatus = false;
  bool valveStatus = false;

  // Temperature control with hysteresis
  if (temperature < (TEMP_SETPOINT - TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, HIGH);
    heaterStatus = true;
    Serial.println("Heater ON");
  } else if (temperature > (TEMP_SETPOINT + TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, LOW);
    heaterStatus = false;
    Serial.println("Heater OFF");
  }

  // pH control
  if (ph > PH_MAX) {
    digitalWrite(VALVE_PIN, HIGH);
    valveStatus = true;
    Serial.println("Valve OPEN (pH too high)");
  } else if (ph < PH_MIN) {
    digitalWrite(VALVE_PIN, LOW);
    valveStatus = false;
    Serial.println("Valve CLOSED (pH too low)");
  }

  // Send telemetry data
  send_telemetry(temperature, ph, heaterStatus, valveStatus);

  tb.loop();
}