
#include "bms_read.hpp"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>

// ── GATT UUIDs ────────────────────────────────────────────────────────────────
static BLEUUID SERVICE_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_RX_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_TX_UUID("0000fff2-0000-1000-8000-00805f9b34fb");

// ── BLE client ────────────────────────────────────────────────────────────────
static BLEClient *ble_client = nullptr;
static BLERemoteCharacteristic *char_rx = nullptr;
static BLERemoteCharacteristic *char_tx = nullptr;
static volatile bool connected = false;
static volatile bool notify_ready = false;

// ── D2 protocol request frame (verified: returns 165-byte response) ──────────
// D2 03 00 00 00 50 56 55  =  addr=0x0000, count=80 registers (160 bytes)
static const uint8_t REQ_D2_STATUS[8] = {0xD2, 0x03, 0x00, 0x00, 0x00, 0x50, 0x56, 0x55};

// ── BLE notification buffer ───────────────────────────────────────────────────
static uint8_t rx_buf[256];
static uint16_t rx_len = 0;


// ── BMS data ─────────────────────────────────────────────────────────────────
static BMSData bms;

// ── Modbus helpers ────────────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static uint16_t u16be(const uint8_t *buf, size_t off)
{
    return ((uint16_t)buf[off] << 8) | buf[off + 1];
}

static bool check_response(const uint8_t *buf, size_t len)
{
    if (len < 5)
        return false;
    if (buf[0] != 0xD2)
        return false; // D2 protocol slave echo
    if (buf[1] != 0x03)
        return false;
    uint8_t n = buf[2];
    if (len < (size_t)(3 + n + 2))
        return false;
    uint16_t stored = (uint16_t)buf[3 + n] | ((uint16_t)buf[3 + n + 1] << 8);
    uint16_t computed = crc16(buf, 3 + n);
    return stored == computed;
}

// ── Decoder (verified offsets) ────────────────────────────────────────────────
static void decode_d2_status(const uint8_t *payload, size_t len)
{
    if (len < 160)
    {
        Serial.printf("[WARN] D2 payload too short: %u bytes (need 160)\n", len);
        return;
    }

    for (int i = 0; i < 4; i++)
        bms.cell_v[i] = u16be(payload, i * 2) * 0.001f;

    bms.total_voltage_v = u16be(payload, 80) * 0.1f;
    bms.current_a = ((int32_t)u16be(payload, 82) - 30000) * 0.1f;
    bms.soc_pct = u16be(payload, 84) * 0.1f;
    bms.max_cell_v = u16be(payload, 86) * 0.001f;
    bms.min_cell_v = u16be(payload, 88) * 0.001f;
    bms.max_bat_temp_c = (int8_t)(u16be(payload, 90) - 40);
    bms.min_bat_temp_c = (int8_t)(u16be(payload, 92) - 40);
    bms.remaining_cap_ah = u16be(payload, 96) * 0.1f;
    bms.cell_count = u16be(payload, 98);
    bms.temp_count = (uint8_t)min((int)u16be(payload, 100), 4);
    bms.cycles = u16be(payload, 102);
    bms.avg_cell_v = u16be(payload, 110) * 0.001f;
    bms.delta_cell_mv = u16be(payload, 112);
    bms.mos_byte = payload[20];
    bms.charge_mos = (bms.mos_byte == 1 || bms.mos_byte == 3);
    bms.discharge_mos = (bms.mos_byte == 2 || bms.mos_byte == 3);

    // Temperatures: payload[64 + i*2], raw-40 = C, skip 0xFF sentinel
    for (uint8_t i = 0; i < bms.temp_count; i++)
    {
        uint16_t raw = u16be(payload, 64 + i * 2);
        bms.temp_c[i] = (raw == 0x00FF) ? -128 : (int8_t)(raw - 40);
    }

    // Rated capacity derived from remaining_Ah / (SOC% / 100)
    // (no separate "rated capacity" field found in this frame layout)
    if (bms.soc_pct > 0.1f){
        bms.rated_cap_ah = bms.remaining_cap_ah / (bms.soc_pct / 100.0f);
    }
    else{
        bms.rated_cap_ah = 0.0f;
    }

    bms.valid = true;

    Serial.printf("[D2] %.1fV  %+.1fA  SOC=%.1f%%  remain=%.1fAh  "
                  "rated~=%.1fAh  cycles=%u  cells=%u  "
                  "max=%.3fV  min=%.3fV  mos=%u(chg=%d,dis=%d)\n",
                  bms.total_voltage_v, bms.current_a, bms.soc_pct,
                  bms.remaining_cap_ah, bms.rated_cap_ah, bms.cycles,
                  bms.cell_count, bms.max_cell_v, bms.min_cell_v,
                  bms.mos_byte, bms.charge_mos, bms.discharge_mos);
}


static void process_rx_buffer()
{
    while (rx_len >= 5)
    {
        if (rx_buf[0] != 0xD2 || rx_buf[1] != 0x03)
        {
            memmove(rx_buf, rx_buf + 1, --rx_len);
            continue;
        }
        uint8_t n = rx_buf[2];
        uint16_t need = 3 + n + 2;
        if (rx_len < need)
            break;

        if (!check_response(rx_buf, need))
        {
            Serial.println("[WARN] CRC mismatch - discarding frame");
            memmove(rx_buf, rx_buf + 1, --rx_len);
            continue;
        }

        decode_d2_status(rx_buf + 3, n);

        rx_len -= need;
        if (rx_len > 0){
            memmove(rx_buf, rx_buf + need, rx_len);
        }
    }
}

static void onNotify(BLERemoteCharacteristic *, uint8_t *data, size_t length, bool)
{
    size_t space = sizeof(rx_buf) - rx_len;
    if (length > space)
    {
        rx_len = 0;
        space = sizeof(rx_buf);
    }
    size_t to_copy = (length < space) ? length : space;
    memcpy(rx_buf + rx_len, data, to_copy);
    rx_len += (uint16_t)to_copy;
    notify_ready = true;
}

class ClientCallbacks : public BLEClientCallbacks
{
    void onConnect(BLEClient *) override
    {
        Serial.println("[BLE] Connected.");
        connected = true;
    }
    void onDisconnect(BLEClient *) override
    {
        Serial.println("[BLE] Disconnected. Will reconnect...");
        connected = false;
        notify_ready = false;
        rx_len = 0;
    }
};
static ClientCallbacks clientCallbacks;

static bool ble_connect()
{
    Serial.printf("[BLE] Connecting to %s ...\n", BMS_ADDRESS);

    if (ble_client != nullptr)
    {
        if (ble_client->isConnected())
            ble_client->disconnect();
        delete ble_client;
        ble_client = nullptr;
    }

    ble_client = BLEDevice::createClient();
    ble_client->setClientCallbacks(&clientCallbacks);

    if (!ble_client->connect(BLEAddress(BMS_ADDRESS)))
    {
        Serial.println("[BLE] Connection failed.");
        return false;
    }

    BLERemoteService *svc = ble_client->getService(SERVICE_UUID);
    if (!svc)
    {
        Serial.println("[BLE] fff0 service not found.");
        ble_client->disconnect();
        return false;
    }

    char_rx = svc->getCharacteristic(CHAR_RX_UUID);
    if (!char_rx || !char_rx->canNotify())
    {
        Serial.println("[BLE] fff1 notify char not found.");
        ble_client->disconnect();
        return false;
    }

    char_tx = svc->getCharacteristic(CHAR_TX_UUID);
    if (!char_tx || !char_tx->canWriteNoResponse())
    {
        Serial.println("[BLE] fff2 write char not found.");
        ble_client->disconnect();
        return false;
    }

    char_rx->registerForNotify(onNotify);
    Serial.println("[BLE] Subscribed, settling...");
    delay(1000);
    return true;
}

static void send_request()
{
    if (!connected || !char_tx)
        return;
    char_tx->writeValue((uint8_t *)REQ_D2_STATUS, 8, false);
}

// ── Print snapshot ────────────────────────────────────────────────────────────
static void print_bms()
{
    if (!bms.valid)
        return;

    Serial.println("==================================================================");
    Serial.printf("  C1:%.3fV  C2:%.3fV  C3:%.3fV  C4:%.3fV\n", bms.cell_v[0], bms.cell_v[1], bms.cell_v[2], bms.cell_v[3]);

    const char *status = bms.current_a > 0.05f ? "Charging" : bms.current_a < -0.05f ? "Discharging"
                                                                                     : "Idle";

    Serial.printf("  Pack:   %.3fV    I: %+.2fA  (%s)    P: %+.1fW\n", bms.total_voltage_v, bms.current_a, status, bms.total_voltage_v * bms.current_a);

    Serial.printf("  SOC:    %.1f%%    Remain: %.1fAh    Rated~: %.1fAh    Cycles: %u\n",
                  bms.soc_pct, bms.remaining_cap_ah, bms.rated_cap_ah, bms.cycles);

    Serial.printf("  Max: %.3fV    Min: %.3fV    Avg: %.3fV    Delta: %u mV\n",
                  bms.max_cell_v, bms.min_cell_v, bms.avg_cell_v, bms.delta_cell_mv);

    Serial.printf("  MOS:  CHG=%s  DIS=%s  (raw=%u)\n",
                  bms.charge_mos ? "ON " : "off",
                  bms.discharge_mos ? "ON " : "off",
                  bms.mos_byte);

    Serial.print("  Temps:  ");
    Serial.printf("BatMax:%dC  BatMin:%dC  ", bms.max_bat_temp_c, bms.min_bat_temp_c);
    for (uint8_t i = 0; i < bms.temp_count; i++)
    {
        if (bms.temp_c[i] != -128)
            Serial.printf("T%u:%dC  ", i + 1, bms.temp_c[i]);
    }
    Serial.println();
    Serial.println("==================================================================");
}

extern "C" void BluetoothTask(void *pvParameters)
{

    Serial.println("\n\n=== JK/Daly BMS BLE Reader (D2 protocol, verified offsets) ===");
    Serial.printf("Target: %s\n\n", BMS_ADDRESS);

    memset(&bms, 0, sizeof(bms));
    BLEDevice::init("ESP32_BMS");

    while (true)
    {
        if (!connected)
        {
            if (!ble_connect())
            {
                Serial.println("Retrying in 5s...");
                delay(5000);
                return;
            }
        }

        send_request();

        uint32_t deadline = millis() + RESPONSE_WAIT_MS;
        notify_ready = false;
        while (millis() < deadline)
        {
            if (notify_ready)
            {
                notify_ready = false;
                process_rx_buffer();
            }
            delay(10);
        }

        if (bms.valid)
        {
            print_bms();
        }
        else{
            Serial.println("[INFO] Waiting for data...");
        }

        delay(POLL_INTERVAL_MS);
    }
}


//public function to get the latest BMS data
BMSData getBMSData()
{
    return bms;
}
