#include "EPDS.h"

// ─── Internal helpers ─────────────────────────────────────────────────────────

static bool i2c_writeReg16(uint8_t addr, uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    return (Wire.endTransmission() == 0);
}

static bool i2c_readReg16(uint8_t addr, uint8_t reg, int16_t &out)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)addr, (uint8_t)2) != 2) return false;
    out = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

// ─── Voltage reads ────────────────────────────────────────────────────────────

// INA226 bus voltage: full register, LSB = 1.25 mV
static float ina226_busVoltage(uint8_t addr)
{
    int16_t raw = 0;
    i2c_readReg16(addr, INA226_REG_BUS_VOLTAGE, raw);
    return (float)raw * 1.25e-3f;
}

// INA219 bus voltage: bits [15:3], LSB = 4 mV
static float ina219_busVoltage(uint8_t addr)
{
    int16_t raw = 0;
    i2c_readReg16(addr, INA219_REG_BUS_VOLTAGE, raw);
    return (float)(raw >> 3) * 4.0e-3f;
}

// ─── Public API ───────────────────────────────────────────────────────────────

bool EPDS_init(EPDS &epds)
{
    epds.initialized = false;

    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast-mode

    bool ok = true;

    ok &= i2c_writeReg16(EPDS_INA226_ROCKET_ADDR, INA226_REG_CONFIG,      INA226_CONFIG_DEFAULT);
    ok &= i2c_writeReg16(EPDS_INA226_ROCKET_ADDR, INA226_REG_CALIBRATION, INA226_ROCKET_CAL);

    ok &= i2c_writeReg16(EPDS_INA219_12V_ADDR,    INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);
    ok &= i2c_writeReg16(EPDS_INA219_12V_ADDR,    INA219_REG_CALIBRATION, INA219_12V_CAL);

    ok &= i2c_writeReg16(EPDS_INA219_5V_ADDR,     INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);
    ok &= i2c_writeReg16(EPDS_INA219_5V_ADDR,     INA219_REG_CALIBRATION, INA219_5V_CAL);

    ok &= i2c_writeReg16(EPDS_INA219_3V3_ADDR,    INA219_REG_CONFIG,      INA219_CONFIG_DEFAULT);
    ok &= i2c_writeReg16(EPDS_INA219_3V3_ADDR,    INA219_REG_CALIBRATION, INA219_3V3_CAL);

    epds.initialized = ok;
    return ok;
}

void EPDS_readAll(EPDS &epds)
{
    if (!epds.initialized) return;

    epds.RKT_V  = ina226_busVoltage(EPDS_INA226_ROCKET_ADDR);
    epds.V12_V  = ina219_busVoltage(EPDS_INA219_12V_ADDR);
    epds.V5_V   = ina219_busVoltage(EPDS_INA219_5V_ADDR);
    epds.V3V3_V = ina219_busVoltage(EPDS_INA219_3V3_ADDR);
}