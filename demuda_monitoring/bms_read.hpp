#ifndef __BMS_READ_HPP__
#define __BMS_READ_HPP__

#include <Arduino.h> 

// ── Configuration ─────────────────────────────────────────────────────────────
#define BMS_ADDRESS "70:C1:45:30:11:52"
#define POLL_INTERVAL_MS 30000
#define RESPONSE_WAIT_MS 600
#define CONNECT_TIMEOUT_MS 10000


// ── BMS data structure ────────────────────────────────────────────────────────
struct BMSData
{
    float cell_v[4]; // cell voltages V (only first 4 populated)
    float total_voltage_v;
    float current_a; // +ve = charging, -ve = discharging
    float soc_pct;
    float remaining_cap_ah;
    float rated_cap_ah; // derived: remaining / (soc/100)
    float max_cell_v;
    float min_cell_v;
    int8_t max_bat_temp_c;
    int8_t min_bat_temp_c;
    int8_t temp_c[4]; // up to 4 temp sensors
    uint8_t temp_count;
    uint16_t cell_count;
    uint16_t cycles;
    float avg_cell_v;
    uint16_t delta_cell_mv;
    uint8_t mos_byte;
    bool charge_mos;
    bool discharge_mos;
    bool valid;
};

BMSData getBMSData();

// ── Task Declaration ──
#ifdef __cplusplus
extern "C" {
#endif
void BluetoothTask(void* pvParameters); 
#ifdef __cplusplus
}
#endif

#endif // __BMS_READ_HPP__