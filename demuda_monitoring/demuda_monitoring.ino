#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include "WiFi_MQTT.h"

WiFiClient espClient;
PubSubClient client(espClient);
char mqtt_buffer[1024];
static uint8_t data_interval;
static unsigned long lastPublish = 0;
// ---------------- Modbus ----------------
ModbusMaster node;

// RS485 direction control pin
#define RE_DE 4

void preTransmission() {
  // digitalWrite(RE_DE, HIGH);
}

void postTransmission() {
  // digitalWrite(RE_DE, LOW);
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  JsonDocument doc;

  // 2. Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("data_interval")) {
    data_interval = (uint16_t)doc["data_interval"];
    Serial.println("Data Time Updated: " + String(data_interval));
  }
}
// ---------------- Helpers ----------------
float readReg(uint16_t reg, float scale, bool isSigned = false) {
  uint8_t result = node.readHoldingRegisters(reg, 1);
  if (result != node.ku8MBSuccess) return NAN;

  int16_t raw = node.getResponseBuffer(0);

  if (isSigned) return ((float)raw) / scale;
  return ((uint16_t)raw) / scale;
}

long readRegRaw(uint16_t reg) {
  uint8_t result = node.readHoldingRegisters(reg, 1);
  if (result != node.ku8MBSuccess) return 0;
  return node.getResponseBuffer(0);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);

  // Modbus init (Serial2 example)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16 TX=17
  node.begin(1, Serial2);                   // slave ID = 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.println("WiFi Connected");
  // MQTT
  client.setBufferSize(1024);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
  data_interval = 30;
  reconnect();
}

// ---------------- MQTT reconnect ----------------
void reconnect() {

  Serial.println("Reconnecting to MQTT Server..");
  while (!client.connected()) {
    if (client.connect("ESP32_Inverter_Mon", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT connected");
      client.subscribe(SUB_TOPIC);
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

float invert(float v) {
  return -v;
}

// ---------------- Main loop ----------------
void loop() {


  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }
  client.loop();  // keep MQTT alive

  if (millis() - lastPublish > (data_interval * 1000))
  {


    StaticJsonDocument<1024> doc;

    // ========== AC INPUT ==========
    doc["ac_in_v"] = readReg(80, 10);
    doc["ac_in_hz"] = readReg(82, 100);

    // ========== AC OUTPUT ==========
    doc["ac_out_v"] = readReg(88, 10);
    doc["ac_out_hz"] = readReg(90, 100);
    doc["ac_out_va"] = readRegRaw(91);
    doc["ac_out_w"] = readRegRaw(92);
    doc["load_pct"] = readReg(93, 10);
    doc["inv_out_w"] = readRegRaw(94);

    // ========== BATTERY ==========
    doc["bat_v"] = readReg(128, 10);
    doc["bat_a"] = invert(readReg(129, 10, true));

    // ========== GRID ==========
    doc["grid_w"] = readRegRaw(149);

    // ========== CHARGING ==========
    doc["charge_v_const"] = readReg(375, 10);
    doc["charge_v_float"] = readReg(376, 10);
    doc["pv_charge_limit_a"] = readReg(377, 10);

    // ========== PV ==========
    doc["pv_v"] = readReg(624, 10);
    doc["pv_a"] = readReg(625, 100, true);
    doc["pv_w"] = readReg(626, 1, true);
    doc["pv_today_kwh"] = readReg(630, 100);

    // ========== ENERGY COUNTERS ==========
    doc["pv_lifetime_kwh"] = readRegRaw(618);  // simplified raw view

    // Serialize JSON
    memset(mqtt_buffer, 0, 1024);
    size_t n = serializeJson(doc, mqtt_buffer);


  if (!client.connected())
  {
    reconnect();
  }
    // Publish MQTT
    if (client.connected())
    {
      client.publish(mqtt_topic, mqtt_buffer);
    } 
    else
    {
      Serial.println("Client Not Connected");
    }
    
    lastPublish = millis();
    Serial.println(mqtt_buffer);
  }

  delay(1000);  // 5s update cycle
}