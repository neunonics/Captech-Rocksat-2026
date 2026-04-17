#pragma once

#include <Arduino.h>
#include <Wire.h>

// ─── I2C Addresses ────────────────────────────────────────────────────────────
#define EPDS_INA226_ROCKET_ADDR  0x40
#define EPDS_INA219_12V_ADDR     0x41
#define EPDS_INA219_5V_ADDR      0x44
#define EPDS_INA219_3V3_ADDR     0x45

// ─── Register addresses ───────────────────────────────────────────────────────
#define INA226_REG_CONFIG        0x00
#define INA226_REG_BUS_VOLTAGE   0x02
#define INA226_REG_CALIBRATION   0x05

#define INA219_REG_CONFIG        0x00
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_CALIBRATION   0x05

// ─── Sensor config words ──────────────────────────────────────────────────────
#define INA226_CONFIG_DEFAULT    0x4527   // avg=16, 1.1ms, shunt+bus continuous
#define INA219_CONFIG_DEFAULT    0x399F   // 32V range, PGA/8, 12-bit, continuous

// ─── Calibration values ───────────────────────────────────────────────────────
#define INA226_ROCKET_CAL        6400
#define INA219_12V_CAL           10240
#define INA219_5V_CAL            13213
#define INA219_3V3_CAL           13213

// ─── EPDS struct ──────────────────────────────────────────────────────────────
struct EPDS {
    bool  initialized;  // set by EPDS_init(); check before reading

    float RKT_V;        // Rocket rail bus voltage  (INA226 @ 0x40)
    float V12_V;        // 12V rail bus voltage     (INA219 @ 0x41)
    float V5_V;         // 5V  rail bus voltage     (INA219 @ 0x44)
    float V3V3_V;       // 3.3V rail bus voltage    (INA219 @ 0x45)
};

// ─── Public API ───────────────────────────────────────────────────────────────

// Initialise I2C and configure all four sensors.
// Returns true and sets epds.initialized = true on success.
bool EPDS_init(EPDS &epds);

// Read bus voltages from all four rails into the struct.
// Safe no-op if epds.initialized is false.
void EPDS_readAll(EPDS &epds);