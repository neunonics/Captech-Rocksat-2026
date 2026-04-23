// EPDS.h

#pragma once
#include <Wire.h>
#include <stdint.h>

// ─── I2C addresses ────────────────────────────────────────────────────────────
#define INA226_ADDR_ROCKET   0x40
#define INA219_ADDR_12V      0x41
#define INA219_ADDR_5V       0x44
#define INA219_ADDR_3V3      0x45

// ─── Bus voltage offsets ──────────────────────────────────────────────────────
#define BUSV_OFFSET_RKT     -0.055f  
#define BUSV_OFFSET_12V      0.038f
#define BUSV_OFFSET_5V       0.039f
#define BUSV_OFFSET_3V3      0.162f 

// ─── INA226 registers ─────────────────────────────────────────────────────────
#define INA226_REG_CONFIG      0x00
#define INA226_REG_BUS_V       0x02
#define INA226_REG_CALIBRATION 0x05
#define INA226_CONFIG_DEFAULT  0x4527

// ─── INA219 registers ─────────────────────────────────────────────────────────
#define INA219_REG_CONFIG      0x00
#define INA219_REG_BUS_V       0x02
#define INA219_REG_CALIBRATION 0x05
#define INA219_CONFIG_DEFAULT  0x399F

// ─── Shunt / current ratings ──────────────────────────────────────────────────
#define SHUNT_OHMS_ROCKET    0.010f
#define SHUNT_OHMS_12V       0.050f
#define SHUNT_OHMS_5V        0.100f
#define SHUNT_OHMS_3V3       0.100f

#define MAX_AMPS_ROCKET      2.5f
#define MAX_AMPS_12V         2.5f
#define MAX_AMPS_5V          1.0f
#define MAX_AMPS_3V3         1.0f

// ─── EPDS data struct ─────────────────────────────────────────────────────────
struct EPDS {
    float RKT_V;
    float V12_V;
    float V5_V;
    float V3V3_V;
    bool  initialized;
};

// ─── Public API ───────────────────────────────────────────────────────────────
bool EPDS_init(EPDS &epds);
void EPDS_readAll(EPDS &epds);
