#include "EPDS.h"

static uint16_t ina226Cal(float maxAmps, float rShunt)
{
    float lsb = maxAmps / 32768.0f;
    return (uint16_t)(0.00512f / (lsb * rShunt));
}

static uint16_t ina219Cal(float maxAmps, float rShunt)
{
    float lsb = maxAmps / 32768.0f;
    return (uint16_t)(0.04096f / (lsb * rShunt));
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

static bool writeRegister(uint8_t addr, uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    return (Wire.endTransmission() == 0);
}

static bool readRegister(uint8_t addr, uint8_t reg, uint16_t &out)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(true);  // full stop — more reliable on Teensy 4.1
    if (Wire.requestFrom((uint8_t)addr, (uint8_t)2) != 2) return false;
    if (Wire.available() < 2) return false;
    out  = (uint16_t)Wire.read() << 8;
    out |= (uint16_t)Wire.read();
    return true;
}

// ─── Public API ───────────────────────────────────────────────────────────────

bool EPDS_init(EPDS &epds)
{
    epds.initialized = false;

    Wire.begin();
    Wire.setClock(400000);
    delay(10);

    bool ok = true;
    ok &= writeRegister(INA226_ADDR_ROCKET, INA226_REG_CALIBRATION, ina226Cal(MAX_AMPS_ROCKET, SHUNT_OHMS_ROCKET));
    ok &= writeRegister(INA226_ADDR_ROCKET, INA226_REG_CONFIG,      INA226_CONFIG_DEFAULT);

    ok &= writeRegister(INA219_ADDR_12V,    INA219_REG_CALIBRATION, ina219Cal(MAX_AMPS_12V, SHUNT_OHMS_12V));
    ok &= writeRegister(INA219_ADDR_12V,    INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);

    ok &= writeRegister(INA219_ADDR_5V,     INA219_REG_CALIBRATION, ina219Cal(MAX_AMPS_5V,  SHUNT_OHMS_5V));
    ok &= writeRegister(INA219_ADDR_5V,     INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);

    ok &= writeRegister(INA219_ADDR_3V3,    INA219_REG_CALIBRATION, ina219Cal(MAX_AMPS_3V3, SHUNT_OHMS_3V3));
    ok &= writeRegister(INA219_ADDR_3V3,    INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);

    epds.initialized = ok;
    return ok;
}

void EPDS_readAll(EPDS &epds)
{
    if (!epds.initialized) return;

    uint16_t raw = 0;

    if (readRegister(INA226_ADDR_ROCKET, INA226_REG_BUS_V, raw))
        epds.RKT_V = (float)raw * 0.00125f + BUSV_OFFSET_RKT;

    if (readRegister(INA219_ADDR_12V, INA219_REG_BUS_V, raw))
        epds.V12_V = (float)((raw >> 3) * 4) / 1000.0f + BUSV_OFFSET_12V;

    if (readRegister(INA219_ADDR_5V, INA219_REG_BUS_V, raw))
        epds.V5_V  = (float)((raw >> 3) * 4) / 1000.0f + BUSV_OFFSET_5V;

    if (readRegister(INA219_ADDR_3V3, INA219_REG_BUS_V, raw))
        epds.V3V3_V = (float)((raw >> 3) * 4) / 1000.0f + BUSV_OFFSET_3V3;
}