#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include "WiFi_MQTT.h"
#include "bms_read.hpp"


#define MQTT_MAX_BUFFER_SIZE 1024

WiFiClient espClient;
PubSubClient client(espClient);
char mqtt_buffer[MQTT_MAX_BUFFER_SIZE];
static uint8_t data_interval;
static unsigned long lastPublish = 0;
TaskHandle_t BTTaskHandle = NULL;
// ---------------- Modbus ----------------
ModbusMaster node;

uint16_t pv_max_power=0, max_load=0;
float max_bat_charge=0, max_bat_discharge=0;

static BMSData bms; // global BMS data structure


template<typename T>
T MAX(T a, T b)
{
    return (a > b) ? a : b;
}

template<typename T>
T MIN(T a, T b)
{
    return (a < b) ? a : b;
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


  // Modbus init (on serial2)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16 TX=17
  node.begin(1, Serial2);                   // slave ID = 1

  // WiFi Setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.println("WiFi Connected");
  // MQTT Setup
  client.setBufferSize(MQTT_MAX_BUFFER_SIZE);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
  data_interval = 30;
  reconnect();

  xTaskCreatePinnedToCore(BluetoothTask, "BluetoothTask", 10000, NULL, 2, &BTTaskHandle, 0);
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
    
    max_load = MAX(max_load, (uint16_t)doc["ac_out_w"]);
    doc["max_load"] = max_load;

    // ========== BATTERY ==========
    doc["bat_v"] = readReg(128, 10);
    doc["bat_a"] = invert(readReg(129, 10, true));
    float batt_w = ((float)doc["bat_a"])*((float)doc["bat_v"]);
    max_bat_charge = MAX(max_bat_charge, batt_w);
    max_bat_discharge = MIN(batt_w,max_bat_discharge);
    doc["max_bat_chg_w"] = max_bat_charge;
    doc["max_bat_dischg_w"] = max_bat_discharge;



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

    pv_max_power = MAX(pv_max_power, (uint16_t)doc["pv_w"]);

    doc["pv_today_kwh"] = readReg(630, 100);
    doc["max_pv"] = pv_max_power;

    // ========== ENERGY COUNTERS ==========
     uint32_t pv_month = ((uint32_t)readRegRaw(619) << 16) | (uint16_t)readRegRaw(618);

    uint32_t pv_year = ((uint32_t)readRegRaw(621) << 16) | (uint16_t)readRegRaw(620);

    uint32_t pv_total = ((uint32_t)readRegRaw(623) << 16) | (uint16_t)readRegRaw(622);

    doc["pv_month_kwh"] = pv_month / 100.0;
    doc["pv_year_kwh"]  = pv_year / 100.0;
    doc["pv_total_kwh"] = pv_total / 100.0;

    BMSData bms = getBMSData();
    if(!bms.valid)
    {
      memset(&bms, 0, sizeof(bms));
    }

    doc["bms_total_v"] = bms.total_voltage_v;
    doc["bms_current_a"] = bms.current_a;
    doc["bms_soc_pct"] = bms.soc_pct;
    doc["bms_remaining_ah"] = bms.remaining_cap_ah*2;
    doc["bms_cycle_count"] = bms.cycles;
    doc["cell_1"]=bms.cell_v[0];
    doc["cell_2"]=bms.cell_v[1];
    doc["cell_3"]=bms.cell_v[2];
    doc["cell_4"]=bms.cell_v[3];

    // Serialize JSON
    memset(mqtt_buffer, 0, MQTT_MAX_BUFFER_SIZE);
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